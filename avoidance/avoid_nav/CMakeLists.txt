cmake_minimum_required(VERSION 3.0.2)
project(avoid_nav)
add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp avoid_msgs geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES avoid_nav_core
  CATKIN_DEPENDS roscpp avoid_msgs geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_library(avoid_nav_core src/planner.cpp)
target_link_libraries(avoid_nav_core ${OpenCV_LIBS})

add_executable(camera_adapter src/camera_adapter.cpp)
target_link_libraries(camera_adapter ${catkin_LIBRARIES})
add_dependencies(camera_adapter ${catkin_EXPORTED_TARGETS})

add_executable(lidar_adapter src/lidar_adapter.cpp)
target_link_libraries(lidar_adapter ${catkin_LIBRARIES})
add_dependencies(lidar_adapter ${catkin_EXPORTED_TARGETS})

add_executable(nav_node src/nav_node.cpp)
target_link_libraries(nav_node avoid_nav_core ${catkin_LIBRARIES})
add_dependencies(nav_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/motion_adapter.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS avoid_nav_core LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS camera_adapter lidar_adapter nav_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
