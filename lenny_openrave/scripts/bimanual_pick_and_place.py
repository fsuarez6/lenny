#! /usr/bin/env python
import enum
import time
import random
import itertools
import collections
import numpy as np
import baldor as br
import criutils as cu
import networkx as nx
import raveutils as ru
import robotsp as rtsp
import lkh_solver as lkh
import openravepy as orpy
# Bimanual planner
from lenny_openrave.bimanual import BimanualPlanner


@enum.unique
class Color(enum.Enum):
  BLUE    = (0, 0, 1)
  GREEN   = (0, 1, 0)
  RED     = (1, 0, 0)
  YELLOW  = (1, 1, 0)


np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = 'worlds/bimanual_pick_and_place.env.xml'
if not env.Load(world_xml):
  raise Exception('Failed to load world: {}'.format(world_xml))
robot = env.GetRobot('robot')
# orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
env.SetViewer('qtcoin')
viewer = env.GetViewer()
# Add the plastic bins relative to table_02
bins_table = env.GetKinBody('table_02')
aabb = bins_table.ComputeAABB()
xdim, _, zdim = 2*aabb.extents()
Tabove_table = bins_table.GetTransform()
Tabove_table[:3,3] += [0, -0.1, zdim+br._EPS]
offset = xdim / 2. - 0.175
direction = Tabove_table[:3,0]
placements = [-1, 0, 1]
colors = [Color.RED, Color.GREEN, Color.BLUE]
num_colors = len(colors)
bins = []
for placement,color in zip(placements, colors):
  body = env.ReadKinBodyXMLFile('objects/plastic_bin.kinbody.xml')
  Tbody = np.array(Tabove_table)
  Tbody[:3,3] += offset*direction*placement
  with env:
    body.SetName('{}_bin'.format(color.name.lower()))
    env.Add(body)
    body.SetTransform(Tbody)
  ru.body.set_body_color(body, diffuse=color.value)
  udata = dict()
  udata['color'] = color
  body.SetUserData(udata)
  bins.append(body)
# Additional working objects
num_bins = len(bins)
bin_transforms = {b.GetUserData()['color']: b.GetTransform() for b in bins}
bin_positions = {b.GetUserData()['color']: b.GetTransform()[:3,3] for b in bins}
bin_indices = {b.GetUserData()['color']: i for i,b in enumerate(bins)}
color_indices = {color: i for i,color in enumerate(colors)}
# Add the cubes randomly
num_cubes = 6
zcube = Tabove_table[2,3] + 1e-5 # Small delta to avoid colliding with the table
xx = np.linspace(0.44, 0.74, num=4)
yy = np.linspace(-0.4, 0.4, num=9) - 0.2145
max_num_cubes = len(xx) * len(yy)
## TODO: Remove everything between ##
# Good seeds: [  4,  20,  21,  22,  24,  40,  44,  61,  62,  65,  74,  86,  96,
#              111, 115, 117, 126, 150, 154, 164, 169, 173, 175, 176, 195, 210,
#              211, 218, 224, 228, 234, 272, 276, 288, 291, 298, 300, 311, 317,
#              326, 330, 331, 351, 352, 356, 361, 363, 365, 374, 376, 380, 391,
#              400, 403, 417]
np.random.seed(4)
[env.Remove(b) for b in env.GetBodies() if 'cube' in b.GetName()]
##
cubes = []
indices = np.random.choice(np.arange(max_num_cubes), num_cubes, replace=False)
yaws = (2*np.random.rand(max_num_cubes) - 1) * np.deg2rad(45)
count = itertools.count(1)
for i,(xcube,ycube) in enumerate(itertools.product(xx, yy)):
  if i not in indices:
    continue
  cube = env.ReadKinBodyXMLFile('objects/wood_cube.kinbody.xml')
  yaw = yaws[i]
  Tcube = br.euler.to_transform(0, 0, yaw)
  Tcube[:3,3] = [xcube, ycube, zcube]
  with env:
    cube.SetName('cube_{0:02d}'.format(count.next()))
    env.Add(cube)
    cube.SetTransform(Tcube)
  # Assign randomly a color for the cube
  color = np.random.choice(colors)
  ru.body.set_body_color(cube, diffuse=color.value)
  udata = dict()
  udata['color'] = color
  cube.SetUserData(udata)
  cubes.append(cube)
# Initialize the bimanual planner
bimanual = BimanualPlanner(robot)
bimanual.set_left_manipulator('arm_left_tool0')
bimanual.set_right_manipulator('arm_right_tool0')
bimanual.set_torso_joint('torso_joint_b1')
# Load IKFast
if not bimanual.load_ikfast(freeinc=np.pi/6.):
  raise Exception('Failed to load IKFast')
# Construct the Multi-commodity pickup-and-delivery TSP (m-PDTSP) graph
graph = nx.Graph()
robot_position = robot.GetTransform()[:3,3]
graph.add_node(1, position=robot_position, demand=0)   # Robot home
cubeid_list = []
binid_list = []
for i,cube in enumerate(cubes):
  cube_id = i+2
  bin_id = num_cubes+i+2
  color = cube.GetUserData()['color']
  Ttcp_cube = cube.GetTransform()
  Ttcp_cube[:3,3] += [0, 0, 0.07]
  cube_position = cube.GetTransform()[:3,3]
  graph.add_node(cube_id, position=cube_position, demand=1, color=color,
                                                                Ttcp=Ttcp_cube)
  Ttcp_bin = np.array(bin_transforms[color])
  Ttcp_bin[:3,3] += [0, 0, 0.15]
  bin_position = bin_positions[color]
  graph.add_node(bin_id, position=bin_position, demand=-1, color=color,
                                                                  Ttcp=Ttcp_bin)
  cubeid_list.append(cube_id)
  binid_list.append(bin_id)
# Compute the distances all the node pairs
distfn = rtsp.metric.euclidean_fn
for u,v in itertools.combinations(graph.nodes_iter(), 2):
  dist = distfn(graph.node[u]['position'], graph.node[v]['position'])
  graph.add_edge(u, v, weight=dist)
# Compute the set of valid color combinations for the 'place'
# That is, valid pairs of bins for dropping the objects
tmp_configs = dict()
valid_place_combinations = set()
nonedge = 99.999
for i,j in itertools.combinations(binid_list, 2):
  color_i = graph.node[i]['color']
  color_j = graph.node[j]['color']
  if color_i == color_j:
    # Remove edges between bins of the same color
    graph.add_edge(i, j, weight=nonedge)
    continue
  combination = (color_i, color_j)
  if combination in valid_place_combinations:
    configs = tmp_configs[combination]
    num_configs = len(configs)
  else:
    Ti = graph.node[i]['Ttcp']
    Tj = graph.node[j]['Ttcp']
    qtorso = bimanual.estimate_torso_angle(Ti[:3,3], Tj[:3,3])
    with robot:
      bimanual.set_torso_joint_value(qtorso)
      solutions = bimanual.find_ik_solutions(Ti, Tj)
    num_configs = len(solutions)
    if num_configs > 0:
      valid_place_combinations.add(combination)
      configs = np.zeros((num_configs,robot.GetDOF()))
      configs[:,0] = qtorso
      configs[:,1:] = solutions
      tmp_configs[combination] = configs
  if num_configs > 0:
    graph.add_edge(i, j, configs=configs)
# Now, process the cubes
valid_cube_pairs = set()
for i,j in itertools.combinations(cubeid_list, 2):
  color_i = graph.node[i]['color']
  color_j = graph.node[j]['color']
  if color_i == color_j:
    continue
  Ti = graph.node[i]['Ttcp']
  Tj = graph.node[j]['Ttcp']
  qtorso = bimanual.estimate_torso_angle(Ti[:3,3], Tj[:3,3])
  with robot:
    bimanual.set_torso_joint_value(qtorso)
    crossed = bimanual.lazy_crossed_arms_check(Ti[:3,3], Tj[:3,3])
  solutions = []
  if (color_i,color_j) in valid_place_combinations and not crossed:
    with robot:
      bimanual.set_torso_joint_value(qtorso)
      solutions = bimanual.find_ik_solutions(Ti, Tj)
  elif (color_j,color_i) in valid_place_combinations and crossed:
    with robot:
      bimanual.set_torso_joint_value(qtorso)
      solutions = bimanual.find_ik_solutions(Tj, Ti)
  num_sols = len(solutions)
  if num_sols > 0:
    if crossed:
      valid_cube_pairs.add((j,i))
    else:
      valid_cube_pairs.add((i,j))
    configs = np.zeros((num_sols,robot.GetDOF()))
    configs[:,0] = qtorso
    configs[:,1:] = solutions
    # Update the TSP graph
    graph.add_edge(i, j, weight=0, configs=configs)     # Cubes
    graph.add_edge(i+num_cubes, j+num_cubes, weight=0)  # Bins
# Populate the demand matrix
nodelist = sorted(graph.nodes())
num_nodes = graph.number_of_nodes()
demand = np.zeros((num_nodes,num_colors), dtype=int)
for i in xrange(num_nodes):
  n = nodelist[i]
  node_demand = graph.node[n]['demand']
  if node_demand == 0:
    continue
  color = graph.node[n]['color']
  color_idx = color_indices[color]
  demand[i,color_idx] = node_demand
if not np.allclose(np.zeros(num_colors), np.sum(demand, axis=0)):
  # The demand sum for each column must be zero
  raise Exception('The demand matrix is not feasible')
# m-PDTSP
params = lkh.solver.SolverParameters()
m, n = num_bins, num_cubes
params.problem_file = '/tmp/lkh/m{0}n{1}task.m-pdtsp'.format(m,n)
params.max_trials = 1
params.runs = 3
params.special = True
lkh.parser.write_tsplib(params.problem_file, graph, params, nodelist=nodelist,
                                            demand=demand, capacity=2, depot=0)
tour, info = lkh.solver.lkh_solver(params)

"""
# Reduce the velocity limits
velocity_limits = robot.GetDOFVelocityLimits()
robot.SetDOFVelocityLimits(0.25*velocity_limits)
# Set the viewer camera
Tcam = br.euler.to_transform(*np.deg2rad([-120, 0, 90]))
Tcam[:3,3] = [2.5, 0.25, 2]
viewer.SetCamera(Tcam)
# Find the IK solutions for both arms
Tleft = env.GetKinBody('cube_05').GetTransform()
Tleft[:3,3] += [0, 0, 0.065]
Tright = env.GetKinBody('cube_01').GetTransform()
Tright[:3,3] += [0, 0, 0.065]
# Estimate torso angle
torso_angle = bimanual.estimate_torso_angle(Tleft[:3,3], Tright[:3,3])
# Find closest IK solution
with robot:
  bimanual.set_torso_joint_value(torso_angle)
  solutions = bimanual.find_ik_solutions(Tleft, Tright)
  idx = bimanual.find_closest_config_index(solutions)
  qarms = solutions[idx]
# robot.SetDOFValues(np.hstack((torso_angle, qarms)))
# Plan the bimanual motion
traj = bimanual.plan(np.hstack((torso_angle, qarms)))
robot.GetController().SetPath(traj)
"""
