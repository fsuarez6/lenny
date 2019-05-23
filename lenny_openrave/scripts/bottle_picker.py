#! /usr/bin/env python
import rospy
import numpy as np
import openravepy as orpy

from lenny_openrave.manager import BottleManager
from lenny_openrave.scheduler import PDPScheduler
from lenny_openrave.bimanual import BimanualPlanner


rospy.init_node("bottle_picker")
np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = "worlds/ctai.env.xml"
if not env.Load(world_xml):
  raise Exception("Failed to load world: {}".format(world_xml))
robot = env.GetRobot("robot")
env.SetViewer("qtcoin")

# Bottle manager
bman = BottleManager(env)
bman.update()
bottles = bman.get_bottles()
bins = bman.get_bins()

# Initialize the bimanual planner
bimanual = BimanualPlanner(robot)
bimanual.set_left_manipulator('arm_left_tool0')
bimanual.set_right_manipulator('arm_right_tool0')
bimanual.set_torso_joint('torso_joint_b1')
if not bimanual.load_ikfast(freeinc=np.pi / 6.):
    rospy.logerr("Failed to load IKFast. Run the generate_robot_databases.sh script.")
    exit(0)

# PDP Scheduler
scheduler = PDPScheduler(env, bimanual)
sequence = scheduler.generate_sequence(bins, bottles)

rospy.spin()
