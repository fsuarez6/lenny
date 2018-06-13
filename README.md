# Lenny

ROS packages for the lenny robot.

### Maintainer
* [Francisco Suárez Ruiz](http://fsuarez6.github.io)

## Installation

Go to your ROS working directory:
```bash
cd ~/catkin_ws/src
```

Clone the required repositories:
```bash
git clone https://github.com/fsuarez6/lenny.git
git clone https://github.com/crigroup/handeye.git
git clone https://github.com/crigroup/raveutils.git
git clone https://github.com/crigroup/cri_gazebo_plugins.git
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
```

Install any missing dependencies using rosdep:
```bash
rosdep update
rosdep install --from-paths . --ignore-src -y --rosdistro $ROS_DISTRO
```

Now compile your ROS workspace:
```bash
cd ~/catkin_ws && catkin_make
```

### Testing the Installation

Be sure to always source the appropriate ROS setup file, e.g:
```bash
source ~/catkin_ws/devel/setup.bash
```
You might want to add that line to your `~/.bashrc` file.

Try the following command:
```bash
roslaunch lenny_moveit_config demo.launch
```

## Examples

### Visualization in RViz
```bash
roslaunch lenny_gazebo test_robot_model.launch
```

### ROS-Industrial robot simulator
```bash
roslaunch lenny_control robot_interface_simulator.launch
rviz -d `rospack find lenny_gazebo`/config/robot_state.rviz
```

### Simulation in Gazebo
```bash
roslaunch lenny_gazebo robot_simulator.launch
```

### Bimanual motion planning
```bash
roslaunch lenny_gazebo robot_simulator.launch
roslaunch lenny_control controllers.launch
rosrun lenny_openrave example_bimanual_planning.py
```

### Bimanual trajectory execution
```bash
roslaunch lenny_control robot_interface_simulator.launch
rviz -d `rospack find lenny_gazebo`/config/robot_state.rviz
rosrun lenny_control example_trajectory_controller.py
```

### Camera-robot calibration
```bash
roslaunch lenny_gazebo camera_robot_calibration.launch gui:=false
rosrun lenny_calibration camera_robot_calibration.py
rosrun rqt_service_caller rqt_service_caller
```

```bash
rosrun lenny_gazebo model_tf_broadcaster.py --name openni_kinect --rate 10
rosrun tf static_transform_publisher 0 0 0.035 -1.5708 0 -1.5708 openni_kinect camera_optical_frame 100
```
