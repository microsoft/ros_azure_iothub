# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 2.8.3)
project(roscpp_azure_iothub)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  # WORKAROUND: ros_type_introspection/utils/variant.hpp(235): warning C4267: '=': conversion from 'size_t' to 'uint32_t', possible loss of data
  add_compile_options(/wd4267)
  # WORKAROUND: ros/message_event.h(217): warning C4127: conditional expression is constant
  add_compile_options(/wd4127)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  topic_tools
  ros_type_introspection
  dynamic_reconfigure
)

if(EXISTS "C:/opt/rosdeps/x64/include/azureiot")
  message(STATUS " ******************************")
  message(STATUS " *** Pardon the interuption ***")
  message(STATUS " ******************************")
  message(STATUS "This package has been migrated to use the LTS release of IoT Hub contained in the Vcpkg library manager.")
  message(STATUS "A deprecated install has been detected on this system, which conflicts with the LTS release.")
  message(STATUS "In order to build this package, please perform these one time steps to remove the conflicting install:")
  message(STATUS "del c:\\opt\\rosdeps\\x64\\CMake\\azure_iot*.cmake")
  message(STATUS "del c:\\opt\\rosdeps\\x64\\CMake\\azure_c_shared*.cmake")
  message(STATUS "del c:\\opt\\rosdeps\\x64\\CMake\\uamqp*.cmake")
  message(STATUS "del c:\\opt\\rosdeps\\x64\\CMake\\umqtt*.cmake")
  message(STATUS "del c:\\opt\\rosdeps\\x64\\CMake\\utpm*.cmake")
  message(STATUS "rd /s c:\\opt\\rosdeps\\x64\\include\\azureiot")
  message(STATUS "rosdep update")
  message(STATUS "rosdep install --from-paths src --ignore-src -r -y")

  message(FATAL_ERROR "Once you've completed these one time migration steps, please rebuild. Thank you. ")

endif()

find_package(azure_iot_sdks CONFIG)

if(azure_iot_sdks_FOUND)
  message(STATUS "azure-iot-sdk-c is found - ${IOTHUB_CLIENT_INCLUDES}")
elseif(EXISTS "/usr/include/azureiot")
  if(MSVC)
    set(IOTHUB_CLIENT_INCLUDES "/usr/include/azureiot" "/usr/include/azureiot/inc")
  else()
    set(IOTHUB_CLIENT_INCLUDES "/usr/include/azureiot" "/usr/include/azureiot/inc" "parson")
  endif()
else()
  message(FATAL_ERROR "Cannot locate azure-iot-sdk-c. Please ensure that dependency has been installed and the CMAKE_PREFIX_PATH contains the path to azure_iot_sdksConfig.cmake")
endif()

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

if(MSVC)
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    unofficial::parson::parson
  )
else()
  add_library(parson STATIC
  parson/parson.c
  parson/parson.h
  )
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    parson
  )
endif()


# prefer to use shared library.
if (TARGET iothub_client_dll)
  message(STATUS "using iothub_client_dll")

  target_link_libraries(${PROJECT_NAME}_node
    $<TARGET_LINKER_FILE:iothub_client_dll>
  )
elseif(TARGET iothub_client)
  message(STATUS "using iothub_client static")

  target_link_directories(${PROJECT_NAME}_node
    PRIVATE $<TARGET_LINKER_FILE_DIR:iothub_client>
  )
  target_link_libraries(${PROJECT_NAME}_node
    iothub_client
  )
else()
  message(STATUS "using iot libs")
  target_link_libraries(${PROJECT_NAME}_node
    iothub_client
    iothub_client_mqtt_transport
    umqtt
    aziotsharedutil
    pthread
    curl
    ssl
    crypto
    m
    uuid
    prov_auth_client
    hsm_security_client
    uhttp  )
endif()

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

#############
##  Test   ##
#############

if(WIN32)
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}_testnode test/testiothub.launch test/testiothub.cpp)
    target_link_libraries(${PROJECT_NAME}_testnode iothub_service_client ${catkin_LIBRARIES} )
  endif()
endif()
