#! /usr/bin/env python
import rospy
import numpy as np
import baldor as br
import criutils as cu
import raveutils as ru
import openravepy as orpy
import openravepy as orpy
# Messages
from lenny_msgs.msg import BottleDetection
from lenny_msgs.srv import DetectBottles, DetectBottlesResponse

COLORS = {
    BottleDetection.PET_COLOR: [0.98, 0.85, 0.37, 1.],
    BottleDetection.PET_TRANSPARENT: [0.9, 0.9, 0.9, 0.5],
    BottleDetection.HDPE_COLOR: [0.80392157, 0.36078431, 0.36078431, 1.],
    BottleDetection.HDPE_WHITE: [1., 0.98, 0.98, 1.],
}


class BottleManager(object):
    def __init__(self, env, srv_name="/bottle_detection/update"):
        self.detect_srv = rospy.ServiceProxy(srv_name, DetectBottles)
        rospy.loginfo("Waiting for service: {}".format(srv_name))
        self.detect_srv.wait_for_service()
        # Working entities
        self.env = env
        self.exiting_bottles = dict() # keys: bottle_name, values: bottle_type
        rospy.loginfo("Bottle manager successfully initialized")

    def update(self):
        response = DetectBottlesResponse(success=False)
        try:
            response = self.detect_srv.call()
        except rospy.ServiceException as ex:
            rospy.logwarn("{0} service failed: {1}".format(self.detect_srv.resolved_name, ex))
        if not response.success:
            return
        # Add the new bottles
        new_bottles = set()
        for bottle in response.bottles:
            new_bottles.add(bottle.frame_id)
            parent_body = self.env.GetKinBody(bottle.parent_frame_id)
            if parent_body is None:
                rospy.logwarn("Failed to find parent_frame_id for: {}".format(bottle.frame_id))
                continue
            existing_body = self.env.GetKinBody(bottle.frame_id)
            Tparent = parent_body.GetTransform()
            Tbody_wrt_parent = cu.conversions.from_transform(bottle.transform)
            Tbody = np.dot(Tparent, Tbody_wrt_parent)
            add_new_bottle = False
            if existing_body is None:
                add_new_bottle = True
            elif self.exiting_bottles[bottle.frame_id] != bottle.bottle_type:
                add_new_bottle = True
            else:
                extents = existing_body.GetLinks()[0].GetGeometries()[0].GetBoxExtents()
                if not np.allclose(extents, cu.conversions.from_vector3(bottle.bbox_size)/2.):
                    add_new_bottle = True
                elif not br.transform.are_equal(Tbody, existing_body.GetTransform()):
                    with self.env:
                        existing_body.SetTransform(Tbody)
            if add_new_bottle:
                if existing_body is not None:
                    with self.env:
                        self.env.Remove(existing_body)
                self.exiting_bottles[bottle.frame_id] = bottle.bottle_type
                with self.env:
                    body = orpy.RaveCreateKinBody(self.env, "")
                    body.SetName(bottle.frame_id)
                    boxes = np.zeros(6)
                    boxes[3:] = cu.conversions.from_vector3(bottle.bbox_size) / 2.
                    body.InitFromBoxes(boxes.reshape(1,6), draw=True)
                    ru.body.set_body_color(body, COLORS[bottle.bottle_type][:3])
                    ru.body.set_body_transparency(body, 1.-COLORS[bottle.bottle_type][3])
                    body.SetTransform(Tbody)
                    self.env.Add(body)
        # Remove orphan bottles
        orphan_bottles = set(self.exiting_bottles.keys()).difference(new_bottles)
        [self.env.Remove(orphan) for orphan in orphan_bottles]


rospy.init_node("bottle_picker")
np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = "worlds/ctai.env.xml"
if not env.Load(world_xml):
  raise Exception("Failed to load world: {}".format(world_xml))
robot = env.GetRobot("robot")
env.SetViewer("qtcoin")
bman = BottleManager(env)
bman.update()
rospy.spin()
