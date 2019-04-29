#!/usr/bin/env python
import os
import yaml
import rospy
import rosbag
import argparse
import baldor as br
import criutils as cu
import resource_retriever
# Services
from handeye.srv import CalibrateHandEye, CalibrateHandEyeRequest


def parse_args():
    # Remove extra IPython notebook args
    clean_argv = rospy.myargv()[1:]
    if '-f' in clean_argv:
        clean_argv = clean_argv[2:]
    # Parse
    format_class = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=format_class, description='HandEye calibration from rosbag')
    parser.add_argument('--bag_path', type=str, required=True, help='Path to the rosbag with the calibration poses')
    parser.add_argument('--debug', action='store_true', help='If set, will show debugging messages')
    args = parser.parse_args(clean_argv)
    return args


if __name__ == '__main__':
    args = parse_args()
    log_level= rospy.DEBUG if args.debug else rospy.INFO
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name, log_level=log_level)
    # Read parameters
    setup = cu.read_parameter('~setup', 'Fixed')
    solver = cu.read_parameter('~solver', 'ParkBryan1994')
    ref_frame = cu.read_parameter('~ref_frame', 'torso_base_link')
    cam_frame = cu.read_parameter('~cam_frame', 'camera_optical_frame')
    # Connect to the handeye calibration service
    handeye_srv = rospy.ServiceProxy('handeye_calibration', CalibrateHandEye)
    try:
        handeye_srv.wait_for_service(timeout=2.0)
    except rospy.ROSException:
        srv_name = handeye_srv.resolved_name
        rospy.logerr('Failed to connect to service: {0}'.format(srv_name))
    # Process rosbag
    request = CalibrateHandEyeRequest()
    request.setup = setup
    request.solver = solver
    bag = rosbag.Bag(os.path.expanduser(args.bag_path))
    for topic, msg, t in bag.read_messages():
        if topic == 'ee_poses':
            request.effector_wrt_world.poses.append(msg)
        elif topic == 'marker_poses':
            request.object_wrt_sensor.poses.append(msg)
    # Check bag soundness
    num_samples = len(request.object_wrt_sensor.poses)
    if num_samples != len(request.effector_wrt_world.poses):
        rospy.logerr('Found inconsistency in the input bag')
        exit()
    # Call the handeye Service
    if num_samples >= 4:
        # Send request
        try:
            handeye_srv.wait_for_service(timeout=1.0)
            result = handeye_srv(request)
        except rospy.ROSException:
            srv_name = handeye_srv.resolved_name
            rospy.logerr('Failed to connect to service: {0}'.format(srv_name))
    else:
        rospy.logerr('Num observations {0} < 4'.format(num_samples))
        exit()
    if not result.success:
        rospy.logerr('Handeye calibration failed to converge')
        exit()
    # Write the calibration results
    pose = result.sensor_frame
    rotation = cu.conversions.from_quaternion(pose.orientation)
    translation = cu.conversions.from_point(pose.position)
    T = cu.conversions.from_pose(pose)
    axis,angle,_ = br.transform.to_axis_angle(T)
    yamldata = {'camera_robot': {
                    'parent': ref_frame,
                    'child': cam_frame,
                    'angle_axis': [angle] + axis.flatten().tolist(),
                    'samples': num_samples,
                    'rotation': rotation.flatten().tolist(),
                    'translation': translation.flatten().tolist()},
                        'rmse': {
                            'rotation': result.rotation_rmse,
                            'translation': result.translation_rmse}
                }
    # Write the file
    uri = 'package://lenny_calibration/config/camera_robot_calibration.yaml'
    yaml_path = resource_retriever.get_filename(uri, use_protocol=False)
    # Write YAML file
    filename = os.path.basename(__file__)
    header =  '# This document was autogenerated by the script %s\n' % filename
    header += '# EDITING THIS FILE BY HAND IS NOT RECOMMENDED\n'
    header += '# NOTE 1: This is the transformation between the camera '
    header += 'coordinates and the robot reference system.\n'
    header += '# NOTE 2: The rotation is a quaternion [w,x,y,z]\n'
    with open(yaml_path, 'w') as f:
        f.write(header)
        yaml.safe_dump(yamldata, f)
    rospy.loginfo('Calibration has been written to: {}'.format(yaml_path))
