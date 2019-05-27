#! /usr/bin/env python
import time
import numpy as np
import robotsp as rtsp
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
eman = EnvironmentManager(env, num_cubes=20, seed=123)
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
tgraph, reachable_set = scheduler.construct_pdp_graph(bins, cubes)
nodes = set(tgraph.nodes())
nodes.remove("home")
print ("Can the robot reach all the locations? {0}".format(nodes == reachable_set))
types = EnvironmentManager.COLORS.keys()
sequence = scheduler.generate_sequence(tgraph, types)


import networkx as nx
import IPython
IPython.embed(banner1="")
exit(0)


# Robotsp from home, then to the cubes and finally back home again
qhome = np.zeros(robot.GetActiveDOF())
vmax = robot.GetDOFVelocityLimits(robot.GetActiveDOFIndices())
setslist = [[qhome]]
# Add the cubes and bins configurations.
for cube_i, cube_j in sequence:
    setslist.append(tgraph.edge[cube_i][cube_j]["configs"]) # Cubes
    bin_i = cube_i.replace("cube", "bin")
    bin_j = cube_j.replace("cube", "bin")
    setslist.append(tgraph.edge[bin_i][bin_j]["configs"])   # Bins
setslist += [[qhome]]
cgraph, sets = rtsp.construct.from_sorted_setslist(setslist, distfn=rtsp.metric.max_joint_diff_fn, args=(1./vmax,))
ctour = nx.dijkstra_path(cgraph, source=0, target=cgraph.number_of_nodes()-1)
# Compute the trajectories
cpu_times = []
trajectories = []
for idx in xrange(len(ctour)-1):
    u = ctour[idx]
    v = ctour[idx+1]
    qstart = cgraph.node[u]["value"]
    qgoal = cgraph.node[v]["value"]
    starttime = time.time()
    with robot:
        robot.SetActiveDOFValues(qstart)
        traj = bimanual.plan(qgoal, max_iters=100, max_ppiters=40)
    cputime = time.time() - starttime
    cpu_times.append(cputime)
    trajectories.append(traj)


for u,v in sequence:
    try:
        msg = "{0}: {1}".format(u, tgraph.node[u]["type"])
        print(msg)
        msg = "{0}: {1}".format(v, tgraph.node[v]["type"])
        print(msg)
    except KeyError:
        pass