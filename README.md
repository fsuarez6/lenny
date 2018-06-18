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
First, capture the calibration poses. A good amount of calibration poses is 25:
```bash
roslaunch lenny_calibration capture_poses.launch gui:=false
```
Output example:
```
[INFO] [1529303561.602071, 322.758000]: 25 pose(s) have been written to: ~/.ros/2018-06-18-14-26-11-poses.bag
```

Then, use the generated `rosbag` (`~/.ros/2018-06-18-14-26-11-poses.bag` in the example above) to perform the Handeye calibration:
```bash
roslaunch lenny_calibration camera_robot_calibration.launch           \
  bag_path:=~/.ros/2018-06-18-14-26-11-poses.bag
```
Expected output:
```
[INFO] [1529304246.283849]: Calibration has been written to: ~/ws_lenny/src/lenny/lenny_calibration/config/camera_robot_calibration.yaml
```

Compare the calibration result with the kinect ground truth:
```bash
rosrun lenny_gazebo model_tf_broadcaster.py --name openni_kinect --rate 10
rosrun tf static_transform_publisher 0 0 0.035 -1.5708 0 -1.5708              \
  openni_kinect camera_optical_frame 100
rosrun tf tf_echo torso_base_link camera_optical_frame
```
Example output:
```
At time 13.735
- Translation: [0.535, 0.000, 1.600]
- Rotation: in Quaternion [0.707, -0.707, -0.000, 0.000]
            in RPY (radian) [3.142, -0.000, -1.571]
            in RPY (degree) [179.999, -0.000, -90.000]
```
