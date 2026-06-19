cmake_minimum_required(VERSION 3.0.2)
project(probot_hk_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_commander
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  std_msgs
  sensor_msgs
  probot_driver
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    moveit_commander
    moveit_msgs
    probot_driver
)

catkin_install_python(PROGRAMS
#  scripts/demo_basic.py
#  scripts/demo_collision.py
#  scripts/demo_vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
