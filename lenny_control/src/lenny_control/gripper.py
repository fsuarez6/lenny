#! /usr/bin/env python
import rospy
import actionlib
import threading
import numpy as np
# Gripper action
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
# Link attacher
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class GripperController(object):
  def __init__(self, ns, timeout=5.0, attach_link='lenny::arm_left_link_7_t'):
    # gazebo_ros link attacher
    self.attach_link = attach_link
    self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    rospy.logdebug('Waiting srv: {0}'.format(self.attach_srv.resolved_name))
    rospy.logdebug('Waiting srv: {0}'.format(self.detach_srv.resolved_name))
    self.attach_srv.wait_for_service()
    self.detach_srv.wait_for_service()
    # Gripper action server
    action_server = '{}/gripper_cmd'.format(ns)
    self._client = actionlib.SimpleActionClient(action_server,
                                                          GripperCommandAction)
    self._goal = GripperCommandGoal()
    rospy.logdebug('Waiting for [%s] action server' % action_server)
    server_up = self._client.wait_for_server(timeout=rospy.Duration(timeout))
    if not server_up:
      rospy.logerr('Timed out waiting for Gripper Command'
                   ' Action Server to connect. Start the action server'
                   ' before running this node.')
      raise rospy.ROSException('GripperCommandAction timed out: {0}'.format(action_server))
    rospy.logdebug('Successfully connected to [%s]' % action_server)
    rospy.loginfo('GripperCommandAction initialized')

  def close(self):
    self.command(0.0)

  def command(self, position):
    angle = self.distance_to_angle(position)
    self._goal.command.position = angle
    self._client.send_goal(self._goal)

  def distance_to_angle(self, distance):
    max_gap = 0.085
    distance = np.clip(distance, 0, max_gap)
    angle = (max_gap - distance) * np.deg2rad(46) / max_gap
    return angle

  def get_result(self):
    return self._client.get_result()

  def get_state(self):
    return self._client.get_state()

  def grab(self, link_name):
    parent = self.attach_link.rsplit('::')
    if '::' in link_name:
      divider = '::'
    elif '.' in link_name:
      divider = '.'
    else:
      return False
    child = link_name.rsplit(divider)
    req = AttachRequest()
    req.model_name_1 = parent[0]
    req.link_name_1 = parent[1]
    req.model_name_2 = child[0]
    req.link_name_2 = child[1]
    res = self.attach_srv.call(req)
    return res.ok

  def open(self):
    self.command(0.085)

  def release(self, link_name):
    parent = self.attach_link.rsplit('::')
    if '::' in link_name:
      divider = '::'
    elif '.' in link_name:
      divider = '.'
    else:
      return False
    child = link_name.rsplit(divider)
    req = AttachRequest()
    req.model_name_1 = parent[0]
    req.link_name_1 = parent[1]
    req.model_name_2 = child[0]
    req.link_name_2 = child[1]
    res = self.detach_srv.call(req)
    return res.ok

  def stop(self):
    self._client.cancel_goal()

  def wait(self, timeout=15.0):
    return self._client.wait_for_result(timeout=rospy.Duration(timeout))
