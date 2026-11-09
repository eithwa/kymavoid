cmake_minimum_required(VERSION 3.0.2)
project(avoid_sim)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/field_manager.py scripts/odom_noise.py scripts/virtual_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
