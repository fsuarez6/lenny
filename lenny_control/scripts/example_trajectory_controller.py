#! /usr/bin/env python
import rospy
import numpy as np
# Controller
from lenny_control.controllers import TrajectoryController


if __name__ == '__main__':
  rospy.init_node('example_trajectory_controller')
  controller = TrajectoryController()
  # Set a random goal for the arms
  active_joints = [name for name in controller.joint_names if 'arm' in name]
  controller.set_active_joints(active_joints)
  while not rospy.is_shutdown():
    controller.clear_points()
    qstart = controller.get_active_joint_positions()
    qgoal = 0.25*(2*np.random.rand(len(active_joints)) - 1)
    controller.add_point(qstart, 0.0)
    controller.add_point(qgoal, 3.0)
    controller.start()
    controller.wait()
