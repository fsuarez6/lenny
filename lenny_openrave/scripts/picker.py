#! /usr/bin/env python
import numpy as np
import openravepy as orpy

from lenny_openrave.manager import EnvironmentManager
from lenny_openrave.scheduler import PDPScheduler
from lenny_openrave.bimanual import BimanualPlanner


np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = "worlds/bimanual_pick_and_place.env.xml"
if not env.Load(world_xml):
  raise Exception("Failed to load world: {}".format(world_xml))
robot = env.GetRobot("robot")
robot.SetDOFValues(np.zeros(robot.GetDOF()))
env.SetViewer("qtcoin")

# Environment manager
eman = EnvironmentManager(env, num_cubes=20)
cubes = eman.get_cubes()
bins = eman.get_bins()

# Initialize the bimanual planner
bimanual = BimanualPlanner(robot)
bimanual.set_left_manipulator("arm_left_tool0")
bimanual.set_right_manipulator("arm_right_tool0")
bimanual.set_torso_joint("torso_joint_b1")
if not bimanual.load_ikfast(freeinc=np.pi / 6.):
    print("Failed to load IKFast. Run the generate_robot_databases.sh script.")
    exit(0)

# PDP Scheduler
scheduler = PDPScheduler(bimanual)
print ("Constructing PDP graph...")
graph, reachable_set = scheduler.construct_pdp_graph(bins, cubes)
nodes = set(graph.nodes())
nodes.remove("home")
print ("Can the robot reach all the locations? {0}".format(nodes == reachable_set))
types = EnvironmentManager.COLORS.keys()
# tour = scheduler.generate_sequence(graph, types)



import networkx as nx
import IPython
IPython.embed(banner1="")
exit(0)

import lkh_solver as lkh
# Generate the demand matrix
nodelist = sorted(graph.nodes())
# Make sure that the first node is "home". Seems like the LKH solver is expecting the depot to be the first node.
depot = 0
nodelist.pop(nodelist.index("home"))
nodelist.insert(depot, "home")
num_nodes = graph.number_of_nodes()
num_types = len(types)
type_indices = {name:index for index, name in enumerate(types)}
demand = np.zeros((num_nodes, num_types), dtype=int)
for i in xrange(num_nodes):
    n = nodelist[i]
    node_demand = graph.node[n]["demand"]
    if node_demand == 0:
        continue
    type_name = graph.node[n]["type"]
    type_idx = type_indices[type_name]
    demand[i, type_idx] = node_demand
if np.any(np.sum(demand, axis=0)):
    # The demand sum for each column must be zero
    raise Exception("The demand matrix is not feasible")
# m-PDTSP
params = lkh.solver.SolverParameters()
m, n = num_types, num_nodes - num_types - 1
params.problem_file = "/tmp/lkh/m{0}n{1}task.m-pdtsp".format(m, n)
params.max_trials = 1
params.runs = 3
params.special = True
params.depot = depot + 1
lkh.parser.write_tsplib(params.problem_file, graph, params, nodelist=nodelist, demand=demand, capacity=2, depot=depot)
tour, info = lkh.solver.lkh_solver(params)
if tour is None:
    raise Exception("Failed to find a feasible sequence")
# For the sequence, we only need the cubes pairs
indices = np.array(tour[0]) - 1
if indices[0] != depot:
    raise Exception("The first node must be the depot/home")

sequence = list()
prev_node =  None
for idx in indices:
    node = nodelist[idx]
    if "cube" not in node:
        prev_node = None
        continue
    if prev_node is None:
        prev_node = node
        continue
    sequence.append((prev_node, node))
    weight = graph.edge[prev_node][node]["weight"]
    if weight != 0:
        print("Unexpected weight {0}-{1}: {2}".format(prev_node, node, weight))