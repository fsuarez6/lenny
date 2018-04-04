#! /usr/bin/env python
import time
import numpy as np
import baldor as br
import criutils as cu
import raveutils as ru
import robotsp as rtsp
import openravepy as orpy


np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = 'worlds/bimanual_pick_and_place.env.xml'
if not env.Load(world_xml):
  raise Exception('Failed to load world: {}'.format(world_xml))
robot = env.GetRobot('robot')
# orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
env.SetViewer('qtcoin')
left_manip = robot.GetManipulator('arm_left_tool0')
right_manip = robot.GetManipulator('arm_right_tool0')
if left_manip is None:
  raise Exception('Failed to find manip: arm_left_tool0')
if right_manip is None:
  raise Exception('Failed to find manip: arm_right_tool0')
left_indices = left_manip.GetArmIndices()
right_indices = right_manip.GetArmIndices()
# Load IKFast for each manip
iktype = orpy.IkParameterizationType.Transform6D
robot.SetActiveManipulator(left_manip)
if not ru.kinematics.load_ikfast(robot, iktype):
  raise Exception('Failed to load left IKFast {0}'.format(iktype.name))
robot.SetActiveManipulator(right_manip)
if not ru.kinematics.load_ikfast(robot, iktype):
  raise Exception('Failed to load right IKFast {0}'.format(iktype.name))
# Find the IK solution for both arms
robot.SetActiveManipulator(left_manip)
Tleft = env.GetKinBody('cube03').GetTransform()
Tleft[:3,3] += [0, 0, 0.065]
left_solutions = ru.kinematics.find_ik_solutions(robot, Tleft, iktype,
                                                            collision_free=True)
robot.SetActiveManipulator(right_manip)
Tright = env.GetKinBody('cube01').GetTransform()
Tright[:3,3] += [0, 0, 0.065]
right_solutions = ru.kinematics.find_ik_solutions(robot, Tright, iktype,
                                                            collision_free=True)
if len(left_solutions) == 0 or len(right_solutions) == 0:
  raise Exception('Failed to find IK solutions for both arms')
# Reduce the velocity limits
velocity_limits = robot.GetDOFVelocityLimits()
robot.SetDOFVelocityLimits(0.1*velocity_limits)
# Choose the closest IK solutions
qrobot = np.zeros(robot.GetDOF())
robot.SetDOFValues(qrobot)
min_dist = float('inf')
closest_idx = 0
qd_max = robot.GetDOFVelocityLimits(left_indices)
distances = []
for i, q in enumerate(left_solutions):
  dist = rtsp.metric.max_joint_diff_fn(qrobot[left_indices], q, qd_max)
  distances.append(dist)
  if dist < min_dist:
    min_dist = dist
    closest_idx = i
qleft = left_solutions[closest_idx]
min_dist = float('inf')
closest_idx = 0
qd_max = robot.GetDOFVelocityLimits(right_indices)
distances = []
for i, q in enumerate(right_solutions):
  dist = rtsp.metric.max_joint_diff_fn(qrobot[right_indices], q, qd_max)
  distances.append(dist)
  if dist < min_dist:
    min_dist = dist
    closest_idx = i
qright = right_solutions[closest_idx]

## Plan the bimanual motion
robot.SetDOFValues(qrobot)
t0 = time.time()
params = orpy.Planner.PlannerParameters()
left_spec = left_manip.GetArmConfigurationSpecification()
right_spec = right_manip.GetArmConfigurationSpecification()
params.SetConfigurationSpecification(env, left_spec+right_spec)
params.SetGoalConfig(np.hstack((qleft,qright)))
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
dual_planning_duration = time.time() - t0
if status == orpy.PlannerStatus.HasSolution:
  robot.GetController().SetPath(traj)
  robot.WaitForController(0)

## Plan one arm at the time
robot.SetDOFValues(qrobot)
t0 = time.time()
params = orpy.Planner.PlannerParameters()
left_spec = left_manip.GetArmConfigurationSpecification()
params.SetConfigurationSpecification(env, left_spec)
params.SetGoalConfig(qleft)
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
single_planning_duration = time.time() - t0
if status == orpy.PlannerStatus.HasSolution:
  robot.GetController().SetPath(traj)
  robot.WaitForController(0)
t1 = time.time()
params = orpy.Planner.PlannerParameters()
right_spec = right_manip.GetArmConfigurationSpecification()
params.SetConfigurationSpecification(env, right_spec)
params.SetGoalConfig(qright)
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
single_planning_duration += time.time() - t1
if status == orpy.PlannerStatus.HasSolution:
  robot.GetController().SetPath(traj)
  robot.WaitForController(0)
