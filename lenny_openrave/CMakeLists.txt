cmake_minimum_required(VERSION 2.8.3)
project(lenny_openrave)

find_package(catkin REQUIRED COMPONENTS
  openrave_catkin
)

catkin_python_setup()

catkin_package()

install(DIRECTORY data DESTINATION ${OpenRAVE_DEVEL_DIR})
install(DIRECTORY data DESTINATION ${OpenRAVE_INSTALL_DIR})
