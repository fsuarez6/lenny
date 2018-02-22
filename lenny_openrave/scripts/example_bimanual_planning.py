#! /usr/bin/env python
import signal
import numpy as np
import criutils as cu
import raveutils as ru
import openravepy as orpy

class GracefulDeath:
  """Catch signals to allow graceful shutdown."""
  def __init__(self):
    self.receivedSignal = False
    self.receivedTermSignal = False
    catchSignals = [1, 2, 3, 10, 12, 15]
    for signum in catchSignals:
      signal.signal(signum, self.handler)

  def handler(self, signum, frame):
    self.lastSignal = signum
    self.receivedSignal = True
    if signum in [2, 3, 15]:
      self.receivedTermSignal = True

sighandler = GracefulDeath()
env = orpy.Environment()
robot_xml = 'robots/sda10f.robot.xml'
robot = env.ReadRobotXMLFile(robot_xml)
env.AddRobot(robot)
orpy.RaveSetDebugLevel(orpy.DebugLevel.Fatal)
env.SetViewer('qtcoin')
left_manip = robot.GetManipulator('arm_left_tool0')
right_manip = robot.GetManipulator('arm_right_tool0')
# Run forever-loop
while True:
  if sighandler.receivedTermSignal:
    env.Reset()
    exit()
  # Find a collision free configuration for both arms
  while True:
    config = ru.kinematics.random_joint_values(robot)
    with robot:
      indices = np.hstack((left_manip.GetArmIndices(),right_manip.GetArmIndices()))
      robot.SetDOFValues(config[indices], indices)
      if not env.CheckCollision(robot):
        # Draw the targets
        Tleft = left_manip.GetEndEffectorTransform()
        Tright = right_manip.GetEndEffectorTransform()
        h1 = ru.visual.draw_axes(env, Tleft, dist=0.05)
        h2 = ru.visual.draw_axes(env, Tright, dist=0.05)
        break
  # Planner parameters
  params = orpy.Planner.PlannerParameters()
  left_spec = left_manip.GetArmConfigurationSpecification()
  right_spec = right_manip.GetArmConfigurationSpecification()
  params.SetConfigurationSpecification(env, left_spec+right_spec)
  params.SetGoalConfig(config[indices])
  params.SetMaxIterations(40)
  params.SetPostProcessing('ParabolicSmoother',
                                        '<_nmaxiterations>20</_nmaxiterations>')
  # Start the planner
  traj = orpy.RaveCreateTrajectory(env, '')
  planner = orpy.RaveCreatePlanner(env, 'BiRRT')
  success = planner.InitPlan(None, params)
  if success:
    status = planner.PlanPath(traj)
    if status == orpy.PlannerStatus.HasSolution:
      robot.GetController().SetPath(traj)
      robot.WaitForController(timeout=0)
