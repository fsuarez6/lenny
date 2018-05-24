#! /usr/bin/env python
import math
import itertools
import numpy as np
import raveutils as ru
import robotsp as rtsp
import openravepy as orpy
from shapely.geometry import LineString


class BimanualPlanner(object):
  def __init__(self, robot):
    self.robot = robot
    self.env = self.robot.GetEnv()
    self.joint_names = [j.GetName() for j in self.robot.GetJoints()]
    # Initialize variables
    self.left_manip = None
    self.right_manip = None
    self.left_anchor_joint = None
    self.right_anchor_joint = None
    self.iktype = orpy.IkParameterizationType.Transform6D
    self.torso_joint = None

  def _estimate_arm_lenght(self, manip):
    joints = [j for j in self.robot.GetDependencyOrderedJoints()
                            if j.GetJointIndex() in manip.GetArmIndices()]
    anchor = joints[0].GetAnchor()
    eetrans = manip.GetEndEffectorTransform()[0:3,3]
    armlength = 0
    for j in reversed(joints):
        armlength += math.sqrt(sum((eetrans-j.GetAnchor())**2))
        eetrans = j.GetAnchor()
    return anchor,armlength

  def _get_anchor_joint(self, manip):
    for j in self.robot.GetDependencyOrderedJoints():
      if j.GetJointIndex() in manip.GetArmIndices():
       anchor_joint = j
       break
    return anchor_joint

  def estimate_torso_angle(self, left_point, right_point):
    Tchild = self.torso_joint.GetSecondAttached().GetTransform()
    points = np.vstack((left_point,right_point))
    centroid = np.mean(points, axis=0)
    centroid_wrt_robot = centroid - Tchild[:3,3]
    x,y,_ = centroid_wrt_robot
    angle = np.arctan2(y, x)
    return angle

  def find_closest_config_index(self, configurations, joint_indices=None,
                                                  metric_fn=None, weights=None):
    if joint_indices is None:
      joint_indices = np.hstack((self.left_indices,self.right_indices))
    if metric_fn is None:
      metric_fn = rtsp.metric.max_joint_diff_fn
    if weights is None:
      weights = 1. / self.robot.GetDOFVelocityLimits(joint_indices)
    qrobot = self.robot.GetDOFValues(joint_indices)
    best_idx = None
    min_dist = float('inf')
    for i, q in enumerate(configurations):
      dist = metric_fn(qrobot, q, weights)
      if dist < min_dist:
        min_dist = dist
        best_idx = i
    return best_idx

  def find_ik_solutions(self, Tleft, Tright, collision_free=True,
                                                              lazy_check=False):
    ## Lazy check for reachability
    if lazy_check:
      reach_left = self.lazy_reachability_check(self.left_manip, Tleft[:3,3])
      reach_right = self.lazy_reachability_check(self.right_manip, Tright[:3,3])
      if not (reach_left and reach_right):
        return []
    solutions = []
    # IKFast for the left arm
    self.robot.SetActiveManipulator(self.left_manip)
    left_sols = ru.kinematics.find_ik_solutions(self.robot, Tleft,
                                    self.iktype, collision_free=collision_free)
    if len(left_sols) > 0:
      # IKFast for the right arm
      self.robot.SetActiveManipulator(self.right_manip)
      right_sols = ru.kinematics.find_ik_solutions(self.robot, Tright,
                                    self.iktype, collision_free=collision_free)
      if len(right_sols) > 0:
        # The solutions list is the combinations of left and right sols
        with self.robot:
          indices = np.hstack((self.left_indices,self.right_indices))
          for qleft,qright in itertools.product(left_sols, right_sols):
            config = np.hstack((qleft,qright))
            self.robot.SetDOFValues(config, indices)
            valid_config = True
            if collision_free:
              if self.robot.CheckSelfCollision():
                valid_config = False
            if valid_config:
              solutions.append(config)
    return solutions

  def plan(self, qgoal, max_iters=40, max_ppiters=40):
    traj = orpy.RaveCreateTrajectory(self.env, '')
    params = orpy.Planner.PlannerParameters()
    # Configuration specification
    robot_name = self.robot.GetName()
    torso_idx = self.torso_joint.GetDOFIndex()
    spec = orpy.ConfigurationSpecification()
    spec.AddGroup('joint_values {0} {1}'.format(robot_name, torso_idx), 1, '')
    left_spec = self.left_manip.GetArmConfigurationSpecification()
    right_spec = self.right_manip.GetArmConfigurationSpecification()
    spec += left_spec + right_spec
    params.SetConfigurationSpecification(self.env, spec)
    params.SetGoalConfig(qgoal)
    params.SetMaxIterations(max_iters)
    params.SetPostProcessing('ParabolicSmoother',
                    '<_nmaxiterations>{}</_nmaxiterations>'.format(max_ppiters))
    planner = orpy.RaveCreatePlanner(self.env, 'BiRRT')
    success = planner.InitPlan(None, params)
    status = orpy.PlannerStatus.Failed
    if success:
      status = planner.PlanPath(traj)
    if status != orpy.PlannerStatus.HasSolution:
      traj = None
    return traj

  def lazy_reachability_check(self, manip, point):
    anchor,armlenght = self._estimate_arm_lenght(manip)
    distance = np.linalg.norm(point - anchor)
    reachable = distance < armlenght
    return reachable

  def lazy_crossed_arms_check(self, left_point, right_point):
    left_anchor_pos = self.left_anchor_joint.GetAnchor()
    right_anchor_pos = self.right_anchor_joint.GetAnchor()
    left_line = LineString([left_anchor_pos[:2], left_point[:2]])
    right_line = LineString([right_anchor_pos[:2], right_point[:2]])
    crossed_arms = not left_line.intersection(right_line).is_empty
    return crossed_arms

  def load_ikfast(self, freeinc=np.pi/6.):
    success = False
    from openravepy.databases.inversekinematics import InverseKinematicsModel
    manipulators = [self.left_manip, self.right_manip]
    if None not in manipulators:
      for manip in manipulators:
        ikmodel = InverseKinematicsModel(self.robot, iktype=self.iktype,
                                                                    manip=manip)
        success = ikmodel.load(freeinc=[freeinc])
        if not success:
          break
    return success

  def sample_torso_angles(self, torso_step, lower_limit=None, upper_limit=None):
    if lower_limit is None:
      lower_limit = float(self.torso_joint.GetLimits()[0])
    if upper_limit is None:
      upper_limit = float(self.torso_joint.GetLimits()[1])
    lower_angles = np.arange(0, lower_limit, torso_step*np.sign(lower_limit))
    upper_angles = np.arange(0, upper_limit, torso_step*np.sign(upper_limit))
    return np.unique(np.hstack((lower_angles,upper_angles)))

  def set_left_manipulator(self, manip_name):
    success = False
    self.left_manip = self.robot.GetManipulator(manip_name)
    if self.left_manip is not None:
      success = True
      self.left_indices = self.left_manip.GetArmIndices()
      self.left_anchor_joint = self._get_anchor_joint(self.left_manip)
    return success

  def set_right_manipulator(self, manip_name):
    success = False
    self.right_manip = self.robot.GetManipulator(manip_name)
    if self.right_manip is not None:
      success = True
      self.right_indices = self.right_manip.GetArmIndices()
      self.right_anchor_joint = self._get_anchor_joint(self.right_manip)
    return success

  def set_torso_joint(self, joint_name):
    success = False
    if joint_name in self.joint_names:
      self.torso_joint = self.robot.GetJoint(joint_name)
      success = True
    return success

  def set_torso_joint_value(self, value):
    idx = self.torso_joint.GetDOFIndex()
    with self.env:
      self.robot.SetDOFValues([value], [idx])
