#! /usr/bin/env python
import rospy
import numpy as np
# Controller
from lenny_control.trajectory import TrajectoryController


if __name__ == '__main__':
  np.set_printoptions(precision=4, suppress=True)
  rospy.init_node('example_trajectory_controller')
  controller = TrajectoryController()
  # Set a random goal for the arms
  joint_names = controller.get_joint_names()
  active_joints = [name for name in joint_names if 'b2' not in name]
  controller.set_active_joints(active_joints)
  rospy.loginfo('Moving all the robot joints 3 times...')
  for i in range(3):
    controller.clear_points()
    qstart = controller.get_active_joint_positions()
    qgoal = 0.25*(2*np.random.rand(len(active_joints)) - 1)
    controller.add_point(qstart, 0.0)
    controller.add_point(qgoal, 3.0)
    controller.start()
    controller.wait()
    error = qgoal - controller.get_active_joint_positions()
    rospy.loginfo('Trajectory {0} error: {1}'.format(i+1, error))
