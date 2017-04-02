cmake_minimum_required(VERSION 2.8.3)
project(robotis_mini_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES darwin_control
#  CATKIN_DEPENDS controller_manager effort_controllers
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
