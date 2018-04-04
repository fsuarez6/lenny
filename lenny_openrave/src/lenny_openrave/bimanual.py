#! /usr/bin/env python
import math
import numpy as np
import raveutils as ru
import robotsp as rtsp
import openravepy as orpy


class Bimanual(object):
  def __init__(self, robot):
    self.robot = robot
    self.env = self.robot.GetEnv()
    self.joint_names = [j.GetName() for j in self.robot.GetJoints()]
    # Initialize variables
    self.iktype = None
    self.left_manip = None
    self.right_manip = None
    self.torso_joint = None

  def load_ikfast(self):
    success = False
    if self.left_manip is not None:
      self.robot.SetActiveManipulator(left_manip)
      success = ru.kinematics.load_ikfast(self.robot, iktype)
    if self.right_manip is not None and success:
      self.robot.SetActiveManipulator(self.right_manip)
      success = ru.kinematics.load_ikfast(self.robot, iktype)
    if success:
      self.iktype = iktype
    return success

  def find_ik_solutions(self, Tleft, Tright, torso_step=np.pi/6.,
                                                          collision_free=True):
    if self.torso_joint is not None and torso_step > 0:
      lower = float(self.torso_joint.GetLimits()[0])
      upper = float(self.torso_joint.GetLimits()[1])
      num_steps = math.ceil((upper - lower) / float(torso_step))
      torso_angles = np.linspace(lower, upper, num_steps)
    left_solutions = []
    right_solutions = []
    for angle in torso_angles:
      solutions = []
      self.robot.SetActiveManipulator(self.left_manip)
      left_sols = ru.kinematics.find_ik_solutions(self.robot, Tleft, self.iktype, collision_free=True)

  def set_left_manipulator(self, manip_name):
    self.left_manip = self.robot.GetManipulator(manip_name)
    self.left_indices = self.left_manip.GetArmIndices()

  def set_right_manipulator(self, manip_name):
    self.right_manip = self.robot.GetManipulator(manip_name)
    self.right_indices = self.right_manip.GetArmIndices()

  def set_torso_joint(self, joint_name):
    success = False
    if joint_name in self.joint_names:
      self.torso_joint = self.robot.GetJoint(joint_name)
      success = True
    return success
