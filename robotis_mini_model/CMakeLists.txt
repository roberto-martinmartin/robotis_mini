cmake_minimum_required(VERSION 2.8.3)
project(robotis_mini_model)
find_package(catkin REQUIRED roslaunch)

catkin_package()

roslaunch_add_file_check(launch)
