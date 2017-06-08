cmake_minimum_required(VERSION 2.8.3)
project(sgr_navigation)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  tf2
  tf2_kdl
  costmap_2d
  message_generation
)



catkin_package(
    CATKIN_DEPENDS gmapping
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
