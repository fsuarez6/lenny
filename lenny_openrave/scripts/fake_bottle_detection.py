#! /usr/bin/env python
import os
import rospy
import random
import tf2_ros
import argparse
import threading
import numpy as np
import baldor as br
import criutils as cu
import openravepy as orpy
# Messages and services
from geometry_msgs.msg import Vector3
from lenny_msgs.msg import BottleDetection, BottleDetectionArray
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


# Bottles types and dimensions
PET_TYPES = [BottleDetection.PET_COLOR, BottleDetection.PET_TRANSPARENT]
PET_DIMENSIONS = [  np.array([0.223, 0.065, 0.065]), # 0.5 L: Height: 223 mm, Diameter: 65 mm
                    np.array([0.278, 0.079, 0.079]), # 1.0 L: Height: 278 mm, Diameter: 79 mm
                    np.array([0.320, 0.092, 0.092]), # 1.5 L: Height: 320 mm, Diameter: 92 mm
                    np.array([0.336, 0.101, 0.101]), # 2.0 L: Height: 336 mm, Diameter: 101.5 mm
                 ]
HDPE_TYPES = [BottleDetection.HDPE_COLOR, BottleDetection.HDPE_WHITE]
HDPE_DIMENSIONS = [ np.array([0.125, 0.067, 0.067]), # 0.3 L: Height: 125 mm, Diameter: 67 mm
                    np.array([0.146, 0.077, 0.077]), # 0.5 L: Height: 146 mm, Diameter: 77 mm
                    np.array([0.161, 0.088, 0.088]), # 0.75 L: Height: 161 mm, Diameter: 88 mm
                 ]
BOTTLE_TYPES = PET_TYPES + HDPE_TYPES

# Worktable and bottles placement
WORKTABLE_EXTENTS = np.array([0.515, 0.625])    # The real size is twice the extents
MAX_PLACEMENT_ATTEMPTS = 100


class FakeBottleDetection(object):
    def __init__(self, num_bottles):
        # Thread locking
        self.mutex = threading.Lock()
        # Setup services and topics
        self.shuffle_srv = rospy.Service('fake_bottle_detection/shuffle', Trigger, self.cb_shuffle)
        self.bottles_pub = rospy.Publisher('bottle_detection', BottleDetectionArray, queue_size=1)
        # Working objects
        self.num_bottles = num_bottles
        self.msg = BottleDetectionArray()
        self.msg.bottles = [BottleDetection(parent_frame_id="worktable", score=1.0) for _ in range(self.num_bottles)]
        self.cb_shuffle(TriggerRequest())

    def cb_shuffle(self, request):
        self.mutex.acquire()
        response = TriggerResponse(success=True)
        entities = []
        invalid_bottles = set()
        for i in range(self.num_bottles):
            attempt = 0
            valid_bottle = False
            while (not valid_bottle) and attempt < MAX_PLACEMENT_ATTEMPTS:
                attempt += 1
                bottle_type = random.choice(BOTTLE_TYPES)
                if bottle_type in PET_TYPES:
                    bbox = random.choice(PET_DIMENSIONS)
                elif bottle_type in HDPE_TYPES:
                    bbox = random.choice(HDPE_DIMENSIONS)
                else:
                    rospy.logwarn("Unknown bottle type: {}".format(bottle_type))
                    break
                # TODO: Improve the placement and collision checking
                # Find a valid collision-free placement
                new_pos = (2*np.random.rand(2)-1) * WORKTABLE_EXTENTS
                new_radius = np.max(bbox) / 2.
                if np.any(new_radius+np.abs(new_pos) > WORKTABLE_EXTENTS):
                    # Part of the bottle is outside of the table.
                    continue
                in_collision = False
                for (entity_pos, entity_radius) in entities:
                    distance = np.linalg.norm(entity_pos - new_pos)
                    if (entity_radius+new_radius) > distance:
                        in_collision = True
                        break
                if in_collision:
                    # The bottle is in collision with another one.
                    continue
                valid_bottle = True
            # Populate the bottle details
            if not valid_bottle:
                invalid_bottles.add(i)
                continue
            entities.append((new_pos, new_radius))
            self.msg.bottles[i].bottle_type = bottle_type
            self.msg.bottles[i].frame_id = "bottle_{0:02d}".format(i+1)
            self.msg.bottles[i].bbox_size = Vector3(*bbox)
            # Add 0.1 mm to avoid collisions with the worktable
            self.msg.bottles[i].transform.translation = Vector3(new_pos[0], new_pos[1], bbox[2]/2.+1e-4)
            # Generate a random orientation (yaw)
            quat = br.euler.to_quaternion(0, 0, 2*np.pi*random.random())
            self.msg.bottles[i].transform.rotation = cu.conversions.to_quaternion(quat)
        # Done
        self.mutex.release()
        if response.success:
            response.message = "Shuffled {} bottles".format(self.num_bottles - len(invalid_bottles))
        return response

    def execute(self, rate=10):
        ros_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            ros_rate.sleep()
            with self.mutex:
                self.msg.header.stamp = rospy.Time.now()
                self.bottles_pub.publish(self.msg)


def parse_args():
    # Remove extra IPython notebook args
    clean_argv = rospy.myargv()[1:]
    if "-f" in clean_argv:
        clean_argv = clean_argv[2:]
    parser = argparse.ArgumentParser(
                formatter_class=argparse.RawDescriptionHelpFormatter,
                description="Fake bottle detection node",
                fromfile_prefix_chars="@")
    parser.add_argument("--debug", action="store_true",
            help="If set, will show additional debugging information")
    parser.add_argument("--ipython", action="store_true",
            help="If set, will embed an IPython console. Useful for debugging")
    parser.add_argument("--rate", metavar="", type=float, default=5.0,
            help="Rate to publish the fake bottle detection info. default=%(default).1f'")
    parser.add_argument("--num-bottles", metavar="", type=int, default=10,
            help="Rate to publish the fake bottle detection info. default=%(default)d'")
    return parser.parse_args(rospy.myargv()[1:])


if "__main__" == __name__:
    np.set_printoptions(precision=6, suppress=True)
    options = parse_args()
    log_level= rospy.DEBUG if options.debug else rospy.INFO
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name, log_level=log_level)
    detection = FakeBottleDetection(options.num_bottles)
    detection.execute(options.rate)
