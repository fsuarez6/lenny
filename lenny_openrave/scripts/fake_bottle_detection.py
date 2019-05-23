#! /usr/bin/env python
import os
import rospy
import random
import tf2_ros
import argparse
import numpy as np
import baldor as br
import criutils as cu
import openravepy as orpy
# Messages and services
from geometry_msgs.msg import Vector3
from lenny_msgs.msg import BottleDetection
from lenny_msgs.srv import DetectBottles, DetectBottlesResponse


# Bottles types and dimensions
PET_TYPES = [BottleDetection.PET_COLOR]
PET_DIMENSIONS = [np.array([0.223, 0.065, 0.065]),  # 0.5 L: Height: 223 mm, Diameter: 65 mm
                  np.array([0.161, 0.088, 0.088]),  # 0.75 L: Height: 161 mm, Diameter: 88 mm  
                #   np.array([0.278, 0.079, 0.079]),  # 1.0 L: Height: 278 mm, Diameter: 79 mm
                #   np.array([0.320, 0.092, 0.092]),  # 1.5 L: Height: 320 mm, Diameter: 92 mm
                #   np.array([0.336, 0.101, 0.101]),  # 2.0 L: Height: 336 mm, Diameter: 101.5 mm
                  ]
HDPE_TYPES = [BottleDetection.HDPE_COLOR, BottleDetection.HDPE_WHITE]
HDPE_DIMENSIONS = [np.array([0.125, 0.067, 0.067]),  # 0.3 L: Height: 125 mm, Diameter: 67 mm
                   np.array([0.146, 0.077, 0.077]),  # 0.5 L: Height: 146 mm, Diameter: 77 mm
                   np.array([0.161, 0.088, 0.088]),  # 0.75 L: Height: 161 mm, Diameter: 88 mm
                   ]
BOTTLE_TYPES = PET_TYPES + HDPE_TYPES

# Worktable and bottles placement
WORKTABLE_EXTENTS = np.array([0.515, 0.625])    # The real size is twice the extents
MAX_PLACEMENT_ATTEMPTS = 100


class FakeBottleDetection(object):
    def __init__(self, num_bottles):
        # Setup services and topics
        self.shuffle_srv = rospy.Service('bottle_detection/update', DetectBottles, self.cb_detect)
        # Working objects
        self.num_bottles = num_bottles

    def cb_detect(self, request):
        response = DetectBottlesResponse(success=True)
        entities = []
        invalid_bottles = set()
        stamp = rospy.Time.now()
        for i in range(self.num_bottles):
            bottle = BottleDetection(parent_frame_id="bottles_table", score=1.0)
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
                # Find a valid collision-free placement
                new_pos = (2 * np.random.rand(2) - 1) * WORKTABLE_EXTENTS
                new_radius = np.linalg.norm(bbox[:2]) / 2.
                if np.any(new_radius + np.abs(new_pos) > WORKTABLE_EXTENTS):
                    # Part of the bottle is outside of the table.
                    continue
                in_collision = False
                for (entity_pos, entity_radius) in entities:
                    distance = np.linalg.norm(entity_pos - new_pos)
                    if (entity_radius + new_radius) > distance:
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
            bottle.header.stamp = stamp
            bottle.bottle_type = bottle_type
            bottle.frame_id = "bottle_{0:02d}".format(i + 1)
            bottle.bbox_size = Vector3(*bbox)
            # Add 0.1 mm to avoid collisions with the worktable
            bottle.transform.translation = Vector3(new_pos[0], new_pos[1], bbox[2] / 2. + 1e-4)
            # Generate a random orientation (yaw)
            quat = br.euler.to_quaternion(0, 0, 2 * np.pi * random.random())
            bottle.transform.rotation = cu.conversions.to_quaternion(quat)
            response.bottles.append(bottle)
        # Done
        response.message = "Detected {} bottles".format(self.num_bottles - len(invalid_bottles))
        rospy.loginfo(response.message)
        return response


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
    parser.add_argument("--num-bottles", metavar="", type=int, default=10,
                        help="Number of bottles to be generated. default=%(default)d")
    return parser.parse_args(clean_argv)


if "__main__" == __name__:
    np.set_printoptions(precision=6, suppress=True)
    options = parse_args()
    log_level = rospy.DEBUG if options.debug else rospy.INFO
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name, log_level=log_level)
    detection = FakeBottleDetection(options.num_bottles)
    rospy.loginfo("Started node: {}".format(node_name))
    rospy.spin()
