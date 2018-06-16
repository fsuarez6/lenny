#!/usr/bin/env python
import os
import time
import rospy
import rosbag
import argparse
import criutils as cu
from datetime import datetime
# Transformations
import tf2_ros
import baldor as br
# Messages
from geometry_msgs.msg import Pose, PoseArray
# Services
from std_srvs.srv import Trigger, TriggerResponse


class PoseCapturer(object):
  def __init__(self):
    # Read parameters
    self.ref_frame = cu.read_parameter('~ref_frame', 'torso_base_link')
    self.ee_link = cu.read_parameter('~ee_link', 'arm_left_link_tool0')
    self.marker_frame = cu.read_parameter('~marker_frame', 'marker_582')
    self.cam_frame = cu.read_parameter('~cam_frame', 'camera_optical_frame')
    # Initialize the poses msgs
    self.ee_samples = []
    self.marker_samples = []
    # Configure TF listener
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    # Use the current time for the bag path
    now = time.time()
    time_str = datetime.fromtimestamp(now).strftime('%Y-%m-%d-%H-%M-%S')
    self.path = '~/.ros/{}-poses.bag'.format(time_str)
    # Setup service server
    self.save_srv = rospy.Service('/calibration/save', Trigger,
                                                                  self.cb_save)
    self.capture_srv = rospy.Service('/calibration/capture', Trigger,
                                                                self.cb_capture)
    # Report
    srv_name = self.capture_srv.resolved_name
    rospy.loginfo('Service is up and running: {}'.format(srv_name))
    srv_name = self.save_srv.resolved_name
    rospy.loginfo('Service is up and running: {}'.format(srv_name))

  def cb_capture(self, req):
    res = TriggerResponse()
    srv_name = self.capture_srv.resolved_name
    rospy.logdebug('Called service: {}'.format(srv_name))
    # Read end-effector pose with respect to the reference frame
    ee_pose = self.get_pose_from_tf(self.ref_frame, self.ee_link)
    # Read object (i.e. aruco marker) pose with respect to the camera frame
    marker_pose = self.get_pose_from_tf(self.cam_frame, self.marker_frame)
    if (ee_pose is not None) and (marker_pose is not None):
      self.ee_samples.append(ee_pose)
      self.marker_samples.append(marker_pose)
      res.success = True
      res.message = 'Added samples: {0}'.format(self.get_num_samples())
    return res

  def cb_save(self, req):
    res = TriggerResponse()
    srv_name = self.save_srv.resolved_name
    rospy.logdebug('Called service: {}'.format(srv_name))
    num_samples = self.get_num_samples()
    if num_samples > 0:
      # Save poses as rosbag
      path = self.path
      bag = rosbag.Bag(os.path.expanduser(path), 'w')
      try:
        stamp = rospy.Time.now()
        for ee_pose, marker_pose in zip(self.ee_samples, self.marker_samples):
          bag.write('ee_poses', ee_pose, stamp)
          bag.write('marker_poses', marker_pose, stamp)
          stamp += rospy.Duration(1.0)    # Add one second between poses
        res.success = True
        msg = '{0} pose(s) have been written to: {1}'.format(num_samples, path)
      except:
        msg = 'Failed to write poses to: {0}'.format(path)
      finally:
        bag.close()
    else:
      msg = 'Please capture poses before saving'
    rospy.loginfo(msg)
    res.message = msg
    return res

  def get_num_samples(self):
    return len(self.ee_samples)

  def get_pose_from_tf(self, parent, child):
    """
    Get the transform between two frames (if available in TF)
    """
    t = rospy.Time()
    try:
      msg = self.tf_buffer.lookup_transform(parent, child, t)
      p = cu.conversions.from_vector3(msg.transform.translation)
      q = cu.conversions.from_quaternion(msg.transform.rotation)
      pose = Pose()
      pose.position = cu.conversions.to_point(p)
      pose.orientation = cu.conversions.to_quaternion(q)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                                              tf2_ros.ExtrapolationException):
      pose = None
    return pose


def parse_args():
  # Remove extra IPython notebook args
  clean_argv = rospy.myargv()[1:]
  if '-f' in clean_argv:
    clean_argv = clean_argv[2:]
  # Parse
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='Capture poses for handeye calibration')
  parser.add_argument('--debug', action='store_true',
    help='If set, will show debugging messages')
  args = parser.parse_args(clean_argv)
  return args


if __name__ == '__main__':
  args = parse_args()
  log_level= rospy.DEBUG if args.debug else rospy.INFO
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name, log_level=log_level)
  capture = PoseCapturer()
  rospy.spin()
