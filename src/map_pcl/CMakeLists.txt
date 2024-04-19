cmake_minimum_required(VERSION 3.0.2)
project(map_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_to_pointcloud
  src/pcd_to_pointcloud.cpp
)

target_link_libraries(pcd_to_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
