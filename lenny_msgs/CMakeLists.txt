cmake_minimum_required(VERSION 2.8.3)
project(lenny_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(
    FILES
    BottleDetection.srv
)

add_service_files(
    FILES
    SetActiveJoints.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()
