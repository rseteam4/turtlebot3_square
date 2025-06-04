cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_square)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES turtlebot3_square
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/move_square.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## (No need to add any other install rules if you only have Python nodes)

