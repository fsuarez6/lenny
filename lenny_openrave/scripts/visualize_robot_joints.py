#! /usr/bin/env python
import criutils as cu
import raveutils as ru
import openravepy as orpy

env = orpy.Environment()
robot_xml = 'robots/sda10f.robot.xml'
robot = env.ReadRobotXMLFile(robot_xml)
env.AddRobot(robot)
env.SetViewer('qtcoin')
prefixes = ['arm_left_', 'arm_right_']
handles = []
for prefix in prefixes:
  manipname = prefix + 'tool0'
  manip = robot.SetActiveManipulator(manipname)
  for idx in manip.GetArmJoints():
    joint = robot.GetJointFromDOFIndex(idx)
    ray = orpy.Ray(joint.GetAnchor(), joint.GetAxis())
    T = ru.conversions.from_ray(ray)
    handles.append(ru.visual.draw_axes(env, T, dist=0.2))
