cmake_minimum_required(VERSION 3.0.2)
project(vi_grab)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  vi_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs vi_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
