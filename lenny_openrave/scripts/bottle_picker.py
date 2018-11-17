#! /usr/bin/env python
import rospy
import numpy as np
import baldor as br
import criutils as cu
import openravepy as orpy
import openravepy as orpy
# Messages
from lenny_msgs.msg import BottleDetectionArray


rospy.init_node("bottle_picker")
np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = "worlds/ctai.env.xml"
if not env.Load(world_xml):
  raise Exception("Failed to load world: {}".format(world_xml))
robot = env.GetRobot("robot")
env.SetViewer("qtcoin")
viewer = env.GetViewer()
bottle_types = dict()

def cb_bottle_detection(msg):
    new_bottles = set()
    for bottle in msg.bottles:
        new_bottles.add(bottle.frame_id)
        parent_body = env.GetKinBody(bottle.parent_frame_id)
        if parent_body is None:
            rospy.logwarn("Failed to find parent_frame_id for: {}".format(bottle.frame_id))
            continue
        existing_body = env.GetKinBody(bottle.frame_id)
        Tparent = parent_body.GetTransform()
        Tbody_wrt_parent = cu.conversions.from_transform(bottle.transform)
        Tbody = np.dot(Tparent, Tbody_wrt_parent)
        add_new_bottle = False
        if existing_body is None:
            add_new_bottle = True
        elif bottle_types[bottle.frame_id] != bottle.bottle_type:
            add_new_bottle = True
        else:
            extents = existing_body.GetLinks()[0].GetGeometries()[0].GetBoxExtents()
            if not np.allclose(extents, cu.conversions.from_vector3(bottle.bbox_size)/2.):
                add_new_bottle = True
            elif not br.transform.are_equal(Tbody, existing_body.GetTransform()):
                with env:
                    existing_body.SetTransform(Tbody)
        if add_new_bottle:
            if existing_body is not None:
                with env:
                    env.Remove(existing_body)
            bottle_types[bottle.frame_id] = bottle.bottle_type
            with env:
                body = orpy.RaveCreateKinBody(env, "")
                body.SetName(bottle.frame_id)
                boxes = np.zeros(6)
                boxes[3:] = cu.conversions.from_vector3(bottle.bbox_size) / 2.
                body.InitFromBoxes(boxes.reshape(1,6), draw=True)
                body.SetTransform(Tbody)
                env.Add(body)
    # Remove orphan bottles
    orphan_bottles = set(bottle_types.keys()).difference(new_bottles)
    [env.Remove(orphan) for orphan in orphan_bottles]

# Subscribe to the bottle_detection topic
rospy.Subscriber("bottle_detection", BottleDetectionArray, cb_bottle_detection)
rospy.spin()
