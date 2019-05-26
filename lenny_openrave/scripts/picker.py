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


import networkx as nx
import IPython
IPython.embed(banner1="")
exit(0)


import raveutils as ru

for name in graph.nodes():
    if "cube" not in name:
        continue
    cube_offset = np.array([0, 0, 0.17])
    Tleft = env.GetKinBody(name).GetTransform()
    Tleft[:3, 3] += cube_offset
    Tright = env.GetKinBody(name).GetTransform()
    Tright[:3, 3] += cube_offset
    qtorso = bimanual.estimate_torso_angle(Tleft[:3, 3], Tright[:3, 3])
    with robot:
        bimanual.set_torso_joint_value(qtorso)
        solutions = bimanual.find_ik_solutions(Tleft, Tright, collision_free=False)
    if len(solutions) > 0:
        bimanual.set_joint_values(np.hstack((qtorso, solutions[0])))
        raw_input("Cube: {}".format(name))

