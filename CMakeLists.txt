cmake_minimum_required(VERSION 2.8.3)
project(axis_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  sensor_msgs
  project11
  rospy
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/axis_tracker.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES posmv
#  CATKIN_DEPENDS geometry_msgs project11_msgs rospy
#  DEPENDS system_lib
)

