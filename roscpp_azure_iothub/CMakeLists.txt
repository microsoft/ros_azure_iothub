cmake_minimum_required(VERSION 2.8.3)
project(roscpp_azure_iothub)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  topic_tools
  ros_type_introspection
)

## System dependencies are found with CMake's conventions
find_package(azure_c_shared_utility REQUIRED)
find_package(azure_iot_sdks REQUIRED)
find_package(umqtt REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs topic_tools ros_type_introspection
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${IOTHUB_CLIENT_INCLUDES}
)

add_executable(${PROJECT_NAME}_node src/ros_azure_iothub_cpp_node.cpp)

link_directories(${catkin_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   iothub_client
   iothub_client_mqtt_ws_transport
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)