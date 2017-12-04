cmake_minimum_required(VERSION 2.8.3)
project(ros_azure_iothub)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros_azure_iothub
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
