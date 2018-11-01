# Lenny

ROS packages for the lenny robot.

### Maintainer
* [Francisco Suárez Ruiz](http://fsuarez6.github.io)

## Requirements
* ROS Kinetic: http://wiki.ros.org/kinetic/Installation
* OpenRAVE: https://github.com/crigroup/openrave-installation

## Installation

Go to your ROS working directory:
```bash
cd ~/catkin_ws/src
```

Clone the required repositories:
```bash
git clone https://github.com/fsuarez6/lenny.git
git clone https://github.com/crigroup/handeye.git
git clone https://github.com/crigroup/robotiq.git
git clone https://github.com/crigroup/raveutils.git
git clone https://github.com/crigroup/openrave_catkin.git
git clone https://github.com/crigroup/cri_gazebo_plugins.git
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
git clone https://github.com/ros-industrial/motoman.git -b kinetic-devel
git clone https://github.com/ros-industrial/industrial_core -b kinetic-devel
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

You might want to add the line above to your `~/.bashrc` file.

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

Then, use the generated `rosbag` (`~/.ros/2018-06-18-14-26-11-poses.bag` in the example above) to perform the Handeye
calibration:

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


### Troubleshooting

#### Gazebo doesn't load/start

The first time you run `gazebo`, it downloads the default models from [this
repo](https://bitbucket.org/osrf/gazebo_models/src/default/). Given that this repo is above 230 MB, it takes a while to
download it. You could download the models before hand and place then in the default location `~/.gazebo/models`

```bash
wget https://bitbucket.org/osrf/gazebo_models/get/9533d5559309.zip -O gazebo_models.zip
unzip gazebo_models.zip
mv osrf-gazebo_models-9533d5559309 ~/.gazebo/models
```

You can test that gazebo is loading the default models with this command:
```
$ gzserver --verbose

Gazebo multi-robot simulator, version 7.0.0
Copyright (C) 2012-2016 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 172.17.0.2
```

If you get this message:
```
[Wrn] [ModelDatabase.cc:339] Getting models from[http://gazebosim.org/models/]. This may take a few seconds.
```

It means that gazebo cannot find the models and it's attempting to download them.


# CTAI Environment

beacon
conveyor
feeder_table
feeder_table_2
mps
table
worktable
xyz_robot
