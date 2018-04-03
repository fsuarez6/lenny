#! /usr/bin/env python
import rospy
import numpy as np
import criutils as cu
import raveutils as ru
import openravepy as orpy
# Controller
from lenny_control.controllers import TrajectoryController


rospy.init_node('example_bimanual_exe')
# Trajectory controllers
controller = TrajectoryController()
active_joints = [name for name in controller.joint_names if 'arm' in name]
controller.set_active_joints(active_joints)
qrobot = controller.get_active_joint_positions()
# OpenRAVE
env = orpy.Environment()
robot_xml = 'robots/sda10f.robot.xml'
robot = env.ReadRobotXMLFile(robot_xml)
env.AddRobot(robot)
orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
env.SetViewer('qtcoin')
left_manip = robot.GetManipulator('arm_left_tool0')
right_manip = robot.GetManipulator('arm_right_tool0')
indices = list(left_manip.GetArmIndices())
indices += list(right_manip.GetArmIndices())
robot.SetActiveDOFs(indices)
robot.SetActiveDOFValues(qrobot)
# Run forever-loop
while not rospy.is_shutdown():
  # Find a collision free configuration for both arms
  while True:
    config = ru.kinematics.random_joint_values(robot)
    with robot:
      robot.SetActiveDOFValues(config)
      if not env.CheckCollision(robot):
        # Draw the targets
        Tleft = left_manip.GetEndEffectorTransform()
        Tright = right_manip.GetEndEffectorTransform()
        h1 = ru.visual.draw_axes(env, Tleft, dist=0.05)
        h2 = ru.visual.draw_axes(env, Tright, dist=0.05)
        break
  # Planner parameters
  params = orpy.Planner.PlannerParameters()
  left_spec = left_manip.GetArmConfigurationSpecification()
  right_spec = right_manip.GetArmConfigurationSpecification()
  params.SetConfigurationSpecification(env, left_spec+right_spec)
  params.SetGoalConfig(config)
  params.SetMaxIterations(80)
  params.SetPostProcessing('ParabolicSmoother',
                                        '<_nmaxiterations>50</_nmaxiterations>')
  # Start the planner
  traj = orpy.RaveCreateTrajectory(env, '')
  planner = orpy.RaveCreatePlanner(env, 'BiRRT')
  success = planner.InitPlan(None, params)
  if success:
    status = planner.PlanPath(traj)
    if status == orpy.PlannerStatus.HasSolution:
      # Convert traj to ROS
      ros_traj = ru.planning.ros_trajectory_from_openrave(robot.GetName(), traj)
      ros_traj.joint_names = list(active_joints)
      controller.set_trajectory(ros_traj)
      # Visualize in OpenRAVE
      robot.GetController().SetPath(traj)
      controller.start()
      controller.wait()
