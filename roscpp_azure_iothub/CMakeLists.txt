# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(roscpp_azure_iothub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
endif()

if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS FALSE)
endif()

if(MSVC)
  add_compile_definitions(NOMINMAX)
endif()

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  # WORKAROUND: ros_type_introspection/utils/variant.hpp(235): warning C4267: '=': conversion from 'size_t' to 'uint32_t', possible loss of data
  add_compile_options(/wd4267)
  # WORKAROUND: ros/message_event.h(217): warning C4127: conditional expression is constant
  add_compile_options(/wd4127)
  # WORKAROUND: parson/parson.c(62): warning C4232: nonstandard extension used: 'parson_malloc'
  add_compile_options(/wd4232)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

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
  set(IOTHUB_CLIENT_INCLUDES "/usr/include/azureiot" "/usr/include/azureiot/inc" "parson")
else()
  message(FATAL_ERROR "Cannot locate azure-iot-sdk-c. Please ensure that dependency has been installed and the CMAKE_PREFIX_PATH contains the path to azure_iot_sdksConfig.cmake")
endif()

add_library(parson SHARED # TODO, difference between static (original) and shared ?
  parson/parson.c
  parson/parson.h
)

###########
## Build ##
###########

include_directories(include)

add_executable(${PROJECT_NAME}_node src/ros_azure_iothub_cpp_node.cpp)

link_directories(${catkin_LIBRARY_DIRS})

# TODO: remove below
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   parson
# )

# TODO: error, package name parson not found
# ament_target_dependencies(${PROJECT_NAME}_node
#   parson)

ament_target_dependencies(${PROJECT_NAME}_node)

# prefer to use shared library.
if (TARGET iothub_client_dll)
  message(STATUS "using iothub_client_dll")

  target_link_libraries(${PROJECT_NAME}_node # TODO: fix target_link_libraries to ament_target_dependencies
    $<TARGET_LINKER_FILE:iothub_client_dll>
  )
elseif(TARGET iothub_client)
  message(STATUS "using iothub_client static")

  target_link_directories(${PROJECT_NAME}_node
    PRIVATE $<TARGET_LINKER_FILE_DIR:iothub_client>
  )
  target_link_libraries(${PROJECT_NAME}_node # TODO: fix target_link_libraries to ament_target_dependencies
    iothub_client
  )
else()
  message(STATUS "using iot libs")
  target_link_libraries(${PROJECT_NAME}_node # TODO: fix target_link_libraries to ament_target_dependencies
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
    parson
    prov_auth_client
    hsm_security_client
    uhttp  )
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# TODO: remove 
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
##  Test   ##
#############

# TODO, remove below 
# if(WIN32)
#   if(CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     add_rostest_gtest(${PROJECT_NAME}_testnode test/testiothub.launch test/testiothub.cpp)
#     target_link_libraries(${PROJECT_NAME}_testnode iothub_service_client ${catkin_LIBRARIES} )
#   endif()
# endif()

if(WIN32)
  if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_testnode test/testiothub.launch test/testiothub.cpp)
    ament_target_dependencies(${PROJECT_NAME}_testnode
      "rclcpp"
      "std_msgs")
    target_link_libraries(${PROJECT_NAME}_testnode iothub_service_client)
  endif()
endif()

ament_package()