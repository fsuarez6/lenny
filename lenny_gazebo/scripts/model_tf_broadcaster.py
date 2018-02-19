#!/usr/bin/env python
import sys
import rospy
import tf2_ros
import argparse
import threading
# Messages
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


class ModelTfBroadcaster():
  def __init__(self, model_name, parent='world', rate=50):
    self.lock = threading.Lock()
    self.model_name = model_name
    broadcaster = tf2_ros.TransformBroadcaster()
    # Subscribe to gazebo model states topic
    self.tf_msg = None
    topic = '/gazebo/model_states'
    rospy.loginfo('Waiting for {0}'.format(topic))
    rospy.Subscriber(topic, ModelStates, callback=self.cb_states, queue_size=1)
    while self.tf_msg is None:
      rospy.sleep(0.1)
    child = model_name
    rate = rospy.Rate(rate)
    rospy.loginfo('Broadcasting transform {0} -> {1}'.format(parent, child))
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.tf_msg.header.frame_id = parent
      self.tf_msg.child_frame_id = child
      broadcaster.sendTransform(self.tf_msg)
      self.lock.release()
      rate.sleep()

  def cb_states(self, msg):
    self.lock.acquire()
    if self.model_name in msg.name:
      idx = msg.name.index(self.model_name)
      self.tf_msg = TransformStamped()
      self.tf_msg.header.stamp = rospy.Time.now()
      self.tf_msg.transform.translation.x = msg.pose[idx].position.x
      self.tf_msg.transform.translation.y = msg.pose[idx].position.y
      self.tf_msg.transform.translation.z = msg.pose[idx].position.z
      self.tf_msg.transform.rotation = msg.pose[idx].orientation
    self.lock.release()


def restricted_rate(x):
  x = float(x)
  if x <= 0.0 or x > 1000.0:
      raise argparse.ArgumentTypeError('%r not in range [0.0, 1000.0]' % x)
  return x

if __name__ == "__main__":
  rospy.init_node('model_tf_broadcaster', anonymous=False)
  parser = argparse.ArgumentParser(
        description='Broadcast the transform of a gazebo model wrt the world',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        fromfile_prefix_chars='@')
  parser.add_argument('--name', type=str, required=True,
                                                  help='The gazebo model name')
  parser.add_argument('--rate', type=restricted_rate, default= 50,
                            help='Broadcast rate in Hz. default=%(default).1f')
  args = parser.parse_args(rospy.myargv()[1:])
  broadcaster = ModelTfBroadcaster(args.name, rate=args.rate)
