#!/usr/bin/env python
import os
import yaml
import rospy
import argparse
import criutils as cu
import resource_retriever
# Transformations
import tf2_ros
import baldor as br
# Messages
from geometry_msgs.msg import Pose
# Services
from std_srvs.srv import Trigger, TriggerResponse
from handeye.srv import CalibrateHandEye, CalibrateHandEyeRequest


class CameraRobotCalibration(object):
  def __init__(self):
    # Read parameters
    self.ref_frame = cu.read_parameter('~ref_frame', 'torso_base_link')
    self.ee_link = cu.read_parameter('~ee_link', 'arm_left_link_tool0')
    self.marker_frame = cu.read_parameter('~marker_frame', 'marker_582')
    self.cam_frame = cu.read_parameter('~cam_frame',
                                                  'openni_depth_optical_frame')
    # Initialize the calibration request
    self.request = CalibrateHandEyeRequest()
    self.request.setup = cu.read_parameter('~setup', 'Fixed')
    self.request.solver = cu.read_parameter('~solver', 'ParkBryan1994')
    # Configure TF listener
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    # Setup service server
    self.calibrate_srv = rospy.Service('/calibration/calibrate', Trigger,
                                                              self.cb_calibrate)
    self.add_reading_srv = rospy.Service('/calibration/capture', Trigger,
                                                                self.cb_capture)
    # Connect to the handeye calibration service
    self.handeye_srv = rospy.ServiceProxy('handeye_calibration',
                                                              CalibrateHandEye)
    try:
      self.handeye_srv.wait_for_service(timeout=2.0)
    except rospy.ROSException:
      srv_name = self.handeye_srv.resolved_name
      rospy.logerr('Failed to connect to service: {0}'.format(srv_name))
    # Report
    srv_name = self.add_reading_srv.resolved_name
    rospy.loginfo('Calibration server is up and running: {}'.format(srv_name))
    srv_name = self.calibrate_srv.resolved_name
    rospy.loginfo('Calibration server is up and running: {}'.format(srv_name))

  def cb_calibrate(self, req):
    res = TriggerResponse()
    srv_name = self.calibrate_srv.resolved_name
    rospy.logdebug('Called service: {}'.format(srv_name))
    num_observations = self.get_num_samples()
    min_observations = 4
    if num_observations >= min_observations:
      # Send calibration request
      try:
        self.handeye_srv.wait_for_service(timeout=1.0)
        result = self.handeye_srv(self.request)
      except rospy.ROSException:
        srv_name = self.handeye_srv.resolved_name
        rospy.logerr('Failed to connect to service: {0}'.format(srv_name))
      # For a Fixed setup, sensor_frame contains the estimated pose between the
      # sensor and the world
      success = result.success
      res.success = success
      self.write_calibration_result(result)
    else:
      res.message = 'Num observations {0} < {1}'.format(num_observations,
                                                              min_observations)
    return res

  def cb_capture(self, req):
    res = TriggerResponse()
    srv_name = self.add_reading_srv.resolved_name
    rospy.logdebug('Called service: {}'.format(srv_name))
    # Read end-effector pose with respect to the reference frame
    ee_pose = self.get_pose_from_tf(self.ref_frame, self.ee_link)
    # Read object (i.e. aruco marker) pose with respect to the camera frame
    marker_pose = self.get_pose_from_tf(self.cam_frame, self.marker_frame)
    if (ee_pose is not None) and (marker_pose is not None):
      self.request.effector_wrt_world.poses.append(ee_pose)
      self.request.object_wrt_sensor.poses.append(marker_pose)
      num = len(self.request.object_wrt_sensor.poses)
      res.success = True
      res.message = 'Added observations: {0}'.format(num)
    return res

  def get_num_samples(self):
    return len(self.request.object_wrt_sensor.poses)

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

  def write_calibration_result(self, res, path=None):
    pose = res.sensor_frame
    rotation = cu.conversions.from_quaternion(pose.orientation)
    translation = cu.conversions.from_point(pose.position)
    T = cu.conversions.from_pose(pose)
    axis,angle,_ = br.transform.to_axis_angle(T)
    yamldata = {'camera_robot': {
                  'parent': self.ref_frame,
                  'child': self.cam_frame,
                  'angle_axis': [angle] + axis.flatten().tolist(),
                  'samples': self.get_num_samples(),
                  'rotation': rotation.flatten().tolist(),
                  'translation': translation.flatten().tolist()},
                'rmse': {
                  'rotation': res.rotation_rmse,
                  'translation': res.translation_rmse}
                }
    # Write the file
    if path is None:
      uri = 'package://lenny_calibration/config/camera_robot_calibration.yaml'
      path = resource_retriever.get_filename(uri, use_protocol=False)
    # Write YAML file
    filename = os.path.basename(__file__)
    header =  '# This document was autogenerated by the script %s\n' % filename
    header += '# EDITING THIS FILE BY HAND IS NOT RECOMMENDED\n'
    header += '# NOTE 1: This is the transformation between the camera '
    header += 'coordinates and the robot reference system.\n'
    header += '# NOTE 2: The rotation is a quaternion [w,x,y,z]\n'
    with open(path, 'w') as f:
      f.write(header)
      yaml.safe_dump(yamldata, f)
    rospy.loginfo('Calibration has been written to: {}'.format(path))


def parse_args():
  # Remove extra IPython notebook args
  clean_argv = rospy.myargv()[1:]
  if '-f' in clean_argv:
    clean_argv = clean_argv[2:]
  # Parse
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='HandEye calibration server')
  parser.add_argument('--debug', action='store_true',
    help='If set, will show debugging messages')
  args = parser.parse_args(clean_argv)
  return args


if __name__ == '__main__':
  args = parse_args()
  log_level= rospy.DEBUG if args.debug else rospy.INFO
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name, log_level=log_level)
  calibration = CameraRobotCalibration()
  rospy.spin()
