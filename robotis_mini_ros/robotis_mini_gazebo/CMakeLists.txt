cmake_minimum_required(VERSION 2.8.3)
project(robotis_mini_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  robotis_mini_description
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros 
    hector_gazebo_plugins
  DEPENDS 
    gazebo
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
