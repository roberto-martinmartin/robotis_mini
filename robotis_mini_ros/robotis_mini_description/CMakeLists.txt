cmake_minimum_required(VERSION 2.8.3)
project(robotis_mini_description)
find_package(catkin REQUIRED 
	roslaunch
	robotis_mini_control)

catkin_package()

roslaunch_add_file_check(launch)
