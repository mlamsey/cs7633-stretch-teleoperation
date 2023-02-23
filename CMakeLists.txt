cmake_minimum_required(VERSION 3.0.2)
project(cs7633_project)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation)

catkin_python_setup()

add_service_files(
  FILES
  ControlAction.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS rospy
)

include_directories(
)
