#! /usr/bin/env python
import time
import numpy as np
import baldor as br
import criutils as cu
import raveutils as ru
import openravepy as orpy
# Bimanual planner
from lenny_openrave.bimanual import BimanualPlanner


np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = 'worlds/bimanual_pick_and_place.env.xml'
if not env.Load(world_xml):
  raise Exception('Failed to load world: {}'.format(world_xml))
robot = env.GetRobot('robot')
# orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
env.SetViewer('qtcoin')
# Reduce the velocity limits
velocity_limits = robot.GetDOFVelocityLimits()
robot.SetDOFVelocityLimits(0.1*velocity_limits)
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

self = bimanual
qgoal = np.hstack((torso_angle, qarms))
# Plan the bimanual motion
t0 = time.time()
params = orpy.Planner.PlannerParameters()
# Configuration specification
robot_name = self.robot.GetName()
torso_idx = self.torso_joint.GetDOFIndex()
spec = orpy.ConfigurationSpecification()
spec.AddGroup('joint_values {0} {1}'.format(robot_name, torso_idx), 1, '')
left_spec = self.left_manip.GetArmConfigurationSpecification()
right_spec = self.right_manip.GetArmConfigurationSpecification()
spec += left_spec + right_spec
params.SetConfigurationSpecification(self.env, spec)
params.SetGoalConfig(qgoal)
params.SetMaxIterations(80)
params.SetPostProcessing('ParabolicSmoother',
                                      '<_nmaxiterations>50</_nmaxiterations>')
# Start the planner
traj = orpy.RaveCreateTrajectory(env, '')
planner = orpy.RaveCreatePlanner(env, 'BiRRT')
success = planner.InitPlan(None, params)
status = orpy.PlannerStatus.Failed
if success:
  status = planner.PlanPath(traj)
duration = time.time() - t0
if status == orpy.PlannerStatus.HasSolution:
  robot.GetController().SetPath(traj)
  robot.WaitForController(0)
