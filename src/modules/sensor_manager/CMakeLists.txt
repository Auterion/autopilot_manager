cmake_minimum_required(VERSION 3.5)
project(sensor_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(MAVSDK REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PythonInterp 3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(image_downsampler REQUIRED)
find_package(timing_tools REQUIRED)

#################
# Setup targets #
#################

add_library(sensor-manager SHARED 
  SensorManager.cpp
  TimeSync.cpp)
ament_target_dependencies(sensor-manager
  Eigen3
  image_downsampler
  timing_tools
  sensor_msgs
  px4_msgs
  tf2_ros
)
target_include_directories(sensor-manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/autopilot-manager/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${image_downsampler_INCLUDE_DIRS}
)
target_link_libraries(sensor-manager
  MAVSDK::mavsdk_server_utility
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk_mavlink_passthrough
  MAVSDK::mavsdk
  Eigen3::Eigen
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_downsampler_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${timing_tools_LIBRARIES}
)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3 image_downsampler sensor_msgs px4_msgs tf2_ros)
ament_export_targets(export_sensor-manager HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(sensor-manager)

# Install header files
install(FILES SensorManager.hpp
        DESTINATION include/autopilot-manager
)

# Install artifacts
install(TARGETS sensor-manager
        EXPORT export_sensor-manager
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

############
# Testing ##
############

# TODO
