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
