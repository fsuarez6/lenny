#! /usr/bin/env python
import time
import numpy as np
import baldor as br
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

# Environment manager
eman = EnvironmentManager(env, num_cubes=10, seed=123)
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
# sequence = scheduler.generate_sequence(tgraph, types=EnvironmentManager.COLORS.keys())

# Start and configure the viewer
viewer_name = "qtcoin"
print("Starting viewer: {0}".format(viewer_name))
env.SetViewer(viewer_name)
while (env.GetViewer() is None):
    time.sleep(0.1)
viewer = env.GetViewer()
viewer.SetSize(1024,768)
rpy = np.deg2rad([235., 0., 110.])
Tcamera = br.euler.to_transform(*rpy)
Tcamera[:3,3] = [2.3, 1., 2.1]
viewer.SetCamera(Tcamera)

# # TODO: Move this section to a class
# print ("Solving the problem...")
# import networkx as nx
# import raveutils as ru
# # Robotsp from home, then to the cubes and finally back home again
# qhome = np.zeros(robot.GetActiveDOF())
# vmax = robot.GetDOFVelocityLimits(robot.GetActiveDOFIndices())
# setslist = [[qhome]]
# # Add the cubes and bins configurations.
# for cube_i, cube_j in sequence:
#     setslist.append(tgraph.edge[cube_i][cube_j]["configs"]) # Cubes
#     bin_i = cube_i.replace("cube", "bin")
#     bin_j = cube_j.replace("cube", "bin")
#     setslist.append(tgraph.edge[bin_i][bin_j]["configs"])   # Bins
# setslist += [[qhome]]
# cgraph, sets = rtsp.construct.from_sorted_setslist(setslist, distfn=rtsp.metric.max_joint_diff_fn, args=(1./vmax,))
# ctour = nx.dijkstra_path(cgraph, source=0, target=cgraph.number_of_nodes()-1)
# # Compute the trajectories
# cpu_times = []
# trajectories = []
# for idx in xrange(len(ctour)-1):
#     u = ctour[idx]
#     v = ctour[idx+1]
#     qstart = cgraph.node[u]["value"]
#     qgoal = cgraph.node[v]["value"]
#     starttime = time.time()
#     with robot:
#         robot.SetActiveDOFValues(qstart)
#         traj = bimanual.plan(qgoal, max_iters=60, max_ppiters=40)
#     cpu_times.append(time.time() - starttime)
#     trajectories.append(traj)
# raw_input("Press any key to visualize the trajectories...")
# # Playback the trajectories
# for i, traj in enumerate(trajectories):
#     controller = robot.GetController()
#     controller.SetPath(traj)
#     robot.WaitForController(0)
#     # Delete the cubes for every even trajectory (except for the last one)
#     seq_index, remainder = divmod(i, 2)
#     if (remainder == 0) and (i != len(trajectories) - 1):
#         for name in sequence[seq_index]:
#             with env:
#                 env.RemoveKinBodyByName(name)

import IPython
IPython.embed(banner1="")
exit(0)


for u,v in sequence:
    try:
        msg = "{0}: {1}".format(u, tgraph.node[u]["type"])
        print(msg)
        msg = "{0}: {1}".format(v, tgraph.node[v]["type"])
        print(msg)
    except KeyError:
        pass


Tleft = robot.SetActiveManipulator('arm_left_tool0').GetTransform()
Tright = robot.SetActiveManipulator('arm_right_tool0').GetTransform()
Toffset = np.eye(4)
Toffset[:3,3] = [0., -0.011115027122497567, -0.14]
Tleft = np.dot(Tleft, Toffset)
Tright = np.dot(Tright, Toffset)
h1 = ru.visual.draw_axes(env, Tleft)
h2 = ru.visual.draw_axes(env, Tright)