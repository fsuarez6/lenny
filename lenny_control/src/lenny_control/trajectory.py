#! /usr/bin/env python
import time
import rospy
import actionlib
import threading
import numpy as np
# Messages
from control_msgs.msg import (FollowJointTrajectoryAction,
                                                      FollowJointTrajectoryGoal)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryController(object):
  """
  The `TrajectoryController` uses a `SimpleActionClient` to connect to a
  `joint_trajectory_action` server. Using the action server interface,
  the robot can be controlled by adding points (joint configurations) to a
  `trajectory_msgs/JointTrajectory` object.
  Each point requires the goal position and the goal time. The velocity is
  optional. The acceleration field is `ignored`.

  """
  def __init__(self, timeout=5.0):
    """
    `TrajectoryController` constructor.

    Parameters
    ----------
    timeout: float
      Time in seconds to wait for the `joint_trajectory_action` server
    """
    # Initialize variables
    self.mutex = threading.Lock()
    self.num_active_joints = 0
    self.active_joint_names = []
    self.active_indices = []
    # Wait for the node to connect to roscore
    while rospy.get_time() <= 0:
      time.sleep(0.1)
    # Start the trajectory action client
    action_server = 'joint_trajectory_action'
    self.client = actionlib.SimpleActionClient(action_server,
                                                    FollowJointTrajectoryAction)
    rospy.logdebug('Waiting for [{0}] action server'.format(action_server))
    server_up = self.client.wait_for_server(timeout=rospy.Duration(timeout))
    if not server_up:
      rospy.logerr('Timed out waiting for Joint Trajectory Action Server ' +
                   'to connect. Start the action server before.')
      msg = 'JointTrajectoryController timeout: {0}'.format(action_server)
      raise rospy.ROSException(msg)
    rospy.logdebug('Successfully connected to [{0}]'.format(action_server))
    # Subscribe to the joint_states topic
    param = '/controller_joint_names'
    self.joint_names = rospy.get_param(param, [])
    if len(self.joint_names) == 0:
      msg = 'Failed to find parameter: {}'.format(param)
      rospy.logerr(msg)
      raise rospy.ROSException(msg)
    self.num_joints = len(self.joint_names)
    # Initialize the joint positions to nan (not a number)
    self.joint_positions = np.full(self.num_joints, np.nan, dtype=float)
    rospy.Subscriber('joint_states', JointState, self.cb_joint_states,
                                                                  queue_size=1)
    rospy.logdebug('Waiting for topic: joint_states')
    success = False
    t0 = rospy.get_time()
    while not success:
      success = np.isnan(self.joint_positions).any()
      if rospy.is_shutdown() or (rospy.get_time()-t0) > timeout:
        break
      rospy.sleep(0.01)
    if not success:
      msg = 'Timed out waiting for topic: joint_states'
      rospy.logerr(msg)
      raise rospy.ROSException(msg)
    # Create the goal instance
    self.goal = FollowJointTrajectoryGoal()
    self.goal.trajectory.joint_names = list(self.joint_names)
    # Give it some time for the connections to settle
    rospy.sleep(0.5)
    rospy.loginfo('TrajectoryController successfully initialized')

  def cb_joint_states(self, msg):
    """
    Callback executed every time a message is published in the `joint_states`
    topic.

    Parameters
    ----------
    msg: sensor_msgs/JointState
      The robot joint state message
    """
    with self.mutex:
      for msg_idx,name in enumerate(msg.name):
        if name in self.joint_names:
          idx = self.joint_names.index(name)
          self.joint_positions[idx] = msg.position[msg_idx]

  def add_point(self, positions, duration, velocities=None):
    """
    Add a point to the trajectory. Each point must be specified by the goal
    position and the goal time. The velocity is optional.

    Parameters
    ----------
    positions: array_like
      The goal position in the joint configuration space
    duration: float
      The duration given for the robot to reach the goal position.
    velocities: array_like
      The velocity of arrival at the goal position. If not given, zero-velocity
      is assumed.

    Returns
    -------
    success: bool
      `True` if the point is added, `False` otherwise.

    Note
    ----
    The lenght of the `positions` and `velocities` list must be equal to the
    number of active joints

    See Also
    --------
    TrajectoryController.get_active_joints
    TrajectoryController.set_active_joints
    """
    pos_array = self.get_joint_positions()
    success = False
    point = JointTrajectoryPoint()
    time_from_start = self.get_trajectory_duration()
    point.time_from_start += rospy.Duration(time_from_start+duration)
    if self.num_active_joints > 0:
      vel_array = np.zeros(self.num_joints)
      acc_array = np.zeros(self.num_joints)
      pos_array[self.active_indices] = positions
      point.positions = pos_array.tolist()
      if velocities is not None:
        vel_array[self.active_indices] = velocities
      point.velocities = vel_array.tolist()
      point.accelerations = acc_array.tolist()
      success = True
    if success:
      self.goal.trajectory.points.append(point)
    return success

  def clear_points(self):
    """
    Clear all points in the trajectory.
    """
    self.goal.trajectory.points = []

  def execute_plan(self, plan, wait=True):
    """
    Execute a previously planned path using MoveIt `move_group`.

    Parameters
    ----------
    plan: moveit_msgs/RobotTrajectory
      Motion plan to be executed
    wait:
      If `True`, the function waits synchronously until the plan is executed.
    """
    self.set_trajectory(plan.joint_trajectory)
    self.start()
    if wait:
      self.wait()

  def get_action_result(self):
    """
    Return the result **after** the trajectory execution request has been
    processed. Possible values are:

    - FollowJointTrajectoryResult.SUCCESSFUL = 0
    - FollowJointTrajectoryResult.INVALID_GOAL = -1
    - FollowJointTrajectoryResult.INVALID_JOINTS = -2
    - FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP = -3
    - FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED = -4
    - FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED = -5

    Returns
    -------
    result: int
      The result **after** the trajectory has been processed
    """
    return self.client.get_result()

  def get_action_state(self):
    """
    Return the result **during** the processing of the trajectory execution
    request. Possible values are:

    - GoalStatus.PENDING=0
    - GoalStatus.ACTIVE=1
    - GoalStatus.PREEMPTED=2
    - GoalStatus.SUCCEEDED=3
    - GoalStatus.ABORTED=4
    - GoalStatus.REJECTED=5
    - GoalStatus.PREEMPTING=6
    - GoalStatus.RECALLING=7
    - GoalStatus.RECALLED=8
    - GoalStatus.LOST=9

    Returns
    -------
    result: int
      The state **during** the processing of the trajectory execution request
    """
    return self.client.get_state()

  def get_active_joints(self):
    """
    Return the active joint names.

    Returns
    -------
    active_joint_names: list
      The active joint names.
    """
    return list(self.active_joint_names)

  def get_active_joint_positions(self):
    """
    Return the most recent value of the `active` joint positions received
    through the `joint_states` topic.

    Returns
    -------
    active_joint_positions: array_like
      The `active` joint positions values
    """
    return self.get_joint_positions()[self.active_indices]

  def get_num_points(self):
    """
    Return the number of points that have been added to the trajectory

    Returns
    -------
    num_points: int
      Number of points added to the trajectory
    """
    return len(self.goal.trajectory.points)

  def get_joint_names(self):
    """
    Return the names of all the joints.

    Returns
    -------
    joint_names: list
      All the robot joint names.
    """
    return list(self.joint_names)

  def get_joint_positions(self):
    """
    Return the most recent value of the joint positions received through the
    `joint_states` topic.

    Returns
    -------
    joint_positions: array_like
      The joint positions of the robot
    """
    with self.mutex:
      # Make a thread-safe copy of the joint positions
      joint_positions = np.array(self.joint_positions, copy=True)
    return joint_positions

  def get_trajectory_duration(self):
    """
    Return the total duration (in seconds) of the trajectory loaded in memory

    Returns
    -------
    duration: float
      The trajectory duration (in seconds)
    """
    duration = rospy.Duration(0)
    if self.get_num_points() > 0:
      duration = self.goal.trajectory.points[-1].time_from_start
    return duration.to_sec()

  def set_active_joints(self, joint_names):
    """
    Set the active joint names. The given list of joint names must be a subset
    of all the available robot joints.

    Parameters
    ----------
    joint_names: list
      List of joint names that will be controlled

    Returns
    -------
    success: bool
      `True` if the operation was successful, `False` otherwise.
    """
    success = set(joint_names).issubset(self.joint_names)
    if success:
      self.active_joint_names = list(joint_names)
      self.active_indices = []
      for name in joint_names:
        index = self.joint_names.index(name)
        self.active_indices.append(index)
      self.num_active_joints = len(self.active_indices)
    return success

  def set_trajectory(self, traj):
    """
    Set the goal trajectory directly.

    Parameters
    ----------
    traj: trajectory_msgs/JointTrajectory
      The goal trajectory

    Returns
    -------
    success: bool
      `True` if the trajectory is set successfully, `False` otherwise.
    """
    success = False
    if self.set_active_joints(traj.joint_names):
      self.clear_points()
      pos_array = self.get_joint_positions()
      vel_array = np.zeros(self.num_joints)
      acc_array = np.zeros(self.num_joints)
      for i in xrange(len(traj.points)):
        point = JointTrajectoryPoint()
        pos_array[self.active_indices] = traj.points[i].positions
        vel_array[self.active_indices] = traj.points[i].velocities
        # TODO: The Quintic interpolator seems to have some issues when you
        # specify the acceleration. By setting acc to zero, it uses Cubic.
        # acc_array[self.active_indices] = traj.points[i].accelerations
        point.positions = pos_array.tolist()
        point.velocities = vel_array.tolist()
        point.accelerations = acc_array.tolist()
        point.time_from_start = traj.points[i].time_from_start
        self.goal.trajectory.points.append(point)
      success = True
    return success

  def start(self, delay=0.0):
    """
    Start the trajectory. It sends the `FollowJointTrajectoryGoal` to the action
    server.

    Parameters
    ----------
    delay: float
      Delay (in seconds) before executing the trajectory
    """
    num_points = self.get_num_points()
    if num_points > 0 and self.num_active_joints > 0:
      rospy.logdebug('Executing trajectory with {0} points'.format(num_points))
      stamp = rospy.Time.now() + rospy.Duration(delay)
      self.goal.trajectory.header.stamp = stamp
      self.client.send_goal(self.goal)

  def stop(self):
    """
    Stops an active trajectory. If there is not active trajectory an error will
    be shown in the console.
    """
    self.client.cancel_goal()

  def wait(self, timeout=0.1):
    """
    Wait synchronously (with a timeout) until the trajectory action server gives
    a result.

    Parameters
    ----------
    timeout: float
      The additional time to wait on top of the trajectory duration

    Returns
    -------
    done: bool
      `True` if the goal finished. False if the goal didn't finish within the
      allocated timeout
    """
    duration = self.get_trajectory_duration() + timeout
    done = self.client.wait_for_result(timeout=rospy.Duration(duration))
    if not done:
      self.stop()
      rospy.sleep(0.5)
    return done
