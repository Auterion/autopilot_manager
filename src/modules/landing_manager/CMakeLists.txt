cmake_minimum_required(VERSION 3.5)
project(landing_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(PythonInterp 3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(image_downsampler REQUIRED)
find_package(std_msgs REQUIRED)
find_package(timing_tools REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(voxel_map REQUIRED)
find_package(landing_mapper REQUIRED)

#################
# Setup targets #
#################

add_library(landing-manager SHARED
  LandingManager.cpp
  MapVisualizer.cpp
)
ament_target_dependencies(landing-manager
  Eigen3
  landing_mapper
  image_downsampler
  std_msgs
  timing_tools
  visualization_msgs
)
target_include_directories(landing-manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/autopilot-manager/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${landing_mapper_INCLUDE_DIRS}
)
target_link_libraries(landing-manager
  Eigen3::Eigen
  landing_mapper
  voxel_map
  ${rclcpp_LIBRARIES}
  ${image_downsampler_LIBRARIES}
  ${px4_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${timing_tools_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3 landing_mapper std_msgs visualization_msgs)
ament_export_targets(export_landing-manager HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(landing-manager)

# Install header files
install(FILES LandingManager.hpp
        DESTINATION include/autopilot-manager
)

# Install artifacts
install(TARGETS landing-manager
        EXPORT export_landing-manager
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

############
# Testing ##
############

# TODO
