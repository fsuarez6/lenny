#! /usr/bin/env python
import rospy
import numpy as np
import baldor as br
import criutils as cu
import raveutils as ru
import openravepy as orpy
# Messages
from lenny_msgs.msg import BottleDetection
from lenny_msgs.srv import DetectBottles, DetectBottlesResponse


class BottleManager(object):
    COLORS = {
        BottleDetection.PET_COLOR: [0.98, 0.85, 0.37, 1.],
        # BottleDetection.PET_TRANSPARENT: [0.5, 0.5, 0.5, 1.],
        BottleDetection.HDPE_COLOR: [0.80392157, 0.36078431, 0.36078431, 1.],
        BottleDetection.HDPE_WHITE: [1., 0.98, 0.98, 1.],
    }

    def __init__(self, env, srv_name="/bottle_detection/update"):
        self.detect_srv = rospy.ServiceProxy(srv_name, DetectBottles)
        rospy.loginfo("Waiting for service: {}".format(srv_name))
        self.detect_srv.wait_for_service()
        # Working entities
        self.changed = False
        self.env = env
        self.bins = self._add_bins()    # keys: bin_name, values: bin_type
        self.existing_bottles = dict()  # keys: bottle_name, values: bottle_type
        rospy.loginfo("Bottle manager successfully initialized")
    
    def _add_bins(self):
        bins_table = self.env.GetKinBody("bins_table")
        aabb = bins_table.ComputeAABB()
        xdim, _, zdim = 2 * aabb.extents()
        Tabove_table = bins_table.GetTransform()
        Tabove_table[:3, 3] += [0, -0.1, zdim + br._EPS]
        offset = xdim / 2. - 0.175
        direction = Tabove_table[:3, 0]
        placements = [-1., 0, 1.]
        num = 1
        bins = dict()
        for placement, (color_name, color_value) in zip(placements, self.COLORS.items()):
            body = self.env.ReadKinBodyXMLFile("objects/plastic_bin.kinbody.xml")
            Tbody = np.array(Tabove_table)
            Tbody[:3, 3] += offset * direction * placement
            bin_name = "bin_{0:02d}".format(num)
            with self.env:
                body.SetName(bin_name)
                self.env.Add(body)
                body.SetTransform(Tbody)
            ru.body.set_body_color(body, diffuse=color_value)
            bins[bin_name] = color_name
            num += 1
        return bins

    def get_bins(self):
        return self.bins

    def get_bottles(self):
        return self.existing_bottles

    def has_changed(self):
        return self.changed

    def update(self):
        response = DetectBottlesResponse(success=False)
        try:
            response = self.detect_srv.call()
        except rospy.ServiceException as ex:
            rospy.logwarn("{0} service failed: {1}".format(self.detect_srv.resolved_name, ex))
        if not response.success:
            return
        # Add the new bottles
        self.changed = False
        newexisting_bottles = set()
        for bottle in response.bottles:
            newexisting_bottles.add(bottle.frame_id)
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
            elif self.existing_bottles[bottle.frame_id] != bottle.bottle_type:
                add_new_bottle = True
            else:
                extents = existing_body.GetLinks()[0].GetGeometries()[0].GetBoxExtents()
                if not np.allclose(extents, cu.conversions.from_vector3(bottle.bbox_size)/2.):
                    add_new_bottle = True
                elif not br.transform.are_equal(Tbody, existing_body.GetTransform()):
                    with self.env:
                        existing_body.SetTransform(Tbody)
            if add_new_bottle:
                self.changed = True
                if existing_body is not None:
                    with self.env:
                        self.env.Remove(existing_body)
                self.existing_bottles[bottle.frame_id] = bottle.bottle_type
                with self.env:
                    body = orpy.RaveCreateKinBody(self.env, "")
                    body.SetName(bottle.frame_id)
                    boxes = np.zeros(6)
                    boxes[3:] = cu.conversions.from_vector3(bottle.bbox_size) / 2.
                    body.InitFromBoxes(boxes.reshape(1,6), draw=True)
                    ru.body.set_body_color(body, BottleManager.COLORS[bottle.bottle_type][:3])
                    ru.body.set_body_transparency(body, 1.-BottleManager.COLORS[bottle.bottle_type][3])
                    body.SetTransform(Tbody)
                    self.env.Add(body)
        # Remove orphan bottles
        orphanexisting_bottles = set(self.existing_bottles.keys()).difference(newexisting_bottles)
        for orphan in orphanexisting_bottles:
            try:
                self.env.Remove(orphan)
            except:
                pass