cmake_minimum_required(VERSION 3.5)
project(sensor_manager)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(rclcpp REQUIRED)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(PythonInterp 3 REQUIRED)
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  find_package(eigen3_cmake_module REQUIRED)
endif()
find_package(Eigen3 REQUIRED NO_MODULE)


#################
# Setup targets #
#################

add_library(sensor-manager SHARED SensorManager.cpp)
ament_target_dependencies(sensor-manager Eigen3 sensor_msgs)
target_include_directories(sensor-manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/autopilot-manager/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(sensor-manager
  Eigen3::Eigen
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 sensor_msgs)
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
