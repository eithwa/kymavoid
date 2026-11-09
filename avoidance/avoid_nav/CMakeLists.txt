cmake_minimum_required(VERSION 3.0.2)
project(avoid_nav)
find_package(catkin REQUIRED COMPONENTS roscpp avoid_msgs geometry_msgs nav_msgs sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS roscpp avoid_msgs geometry_msgs nav_msgs sensor_msgs std_msgs)
