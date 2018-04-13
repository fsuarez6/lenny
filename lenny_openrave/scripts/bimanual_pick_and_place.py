#! /usr/bin/env python
import time
import numpy as np
import baldor as br
import criutils as cu
import raveutils as ru
import openravepy as orpy
# Bimanual planner
from lenny_openrave.bimanual import BimanualPlanner

"""
Reachability limits
y = [0.44, 0.84]
"""

class Color(object):
  YELLOW  = (1, 1, 0)
  BLUE    = (0, 0.412, 0.58)
  RED     = (1, 0, 0)

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
# Add the plastic bins relative to the bins table
bins_table = env.GetKinBody('bins_table')
aabb = bins_table.ComputeAABB()
xdim, _, zdim = 2*aabb.extents()
Tabove_table = bins_table.GetTransform()
Tabove_table[2,3] += zdim + br._EPS
offset = xdim / 2. - 0.135
direction = Tabove_table[:3,0]
placements = [-1, 0, 1]
colors = [Color.YELLOW, Color.BLUE, Color.RED]
names = ['yellow_bin', 'blue_bin', 'red_bin']
for name,placement,color in zip(names, placements, colors):
  body = env.ReadKinBodyXMLFile('objects/plastic_bin.kinbody.xml')
  Tbody = np.array(Tabove_table)
  Tbody[:3,3] += offset*direction*placement
  with env:
    body.SetName(name)
    env.Add(body)
    body.SetTransform(Tbody)
  ru.body.set_body_color(body, diffuse=color)
# Add the cubes randomly
#  objects/wood_cube.kinbody.xml
# Reduce the velocity limits
velocity_limits = robot.GetDOFVelocityLimits()
robot.SetDOFVelocityLimits(0.25*velocity_limits)
# Set the viewer camera
Tcam = br.euler.to_transform(*np.deg2rad([-120, 0, 90]))
Tcam[:3,3] = [2.5, 0.25, 2]
viewer.SetCamera(Tcam)
# Initialize the bimanual planner
bimanual = BimanualPlanner(robot)
bimanual.set_left_manipulator('arm_left_tool0')
bimanual.set_right_manipulator('arm_right_tool0')
bimanual.set_torso_joint('torso_joint_b1')
# Load IKFast
if not bimanual.load_ikfast():
  raise Exception('Failed to load IKFast')
# Find the IK solutions for both arms
Tleft = env.GetKinBody('cube02').GetTransform()
Tleft[:3,3] += [0, 0, 0.065]
Tright = env.GetKinBody('cube01').GetTransform()
Tright[:3,3] += [0, 0, 0.065]
# Estimate torso angle
torso_angle = bimanual.estimate_torso_angle(Tleft, Tright)
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
