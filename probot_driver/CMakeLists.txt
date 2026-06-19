cmake_minimum_required(VERSION 3.0.2)
project(probot_driver)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs
    trajectory_msgs
    control_msgs
    actionlib
)

catkin_install_python(PROGRAMS
  scripts/hk_driver_node.py
 ## scripts/test_connection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
