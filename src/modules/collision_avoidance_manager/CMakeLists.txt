cmake_minimum_required(VERSION 3.5)
project(collision_avoidance_manager)

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


#################
# Setup targets #
#################

add_library(collision-avoidance-manager SHARED CollisionAvoidanceManager.cpp)
ament_target_dependencies(collision-avoidance-manager
  Eigen3
  image_downsampler
)
target_include_directories(collision-avoidance-manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/autopilot-manager/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(collision-avoidance-manager
  Eigen3::Eigen
  ${rclcpp_LIBRARIES}
  ${image_downsampler_LIBRARIES}
)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3)
ament_export_targets(export_collision-avoidance-manager HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(collision-avoidance-manager)

# Install header files
install(FILES CollisionAvoidanceManager.hpp
        DESTINATION include/autopilot-manager
)

# Install artifacts
install(TARGETS collision-avoidance-manager
        EXPORT export_collision-avoidance-manager
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

############
# Testing ##
############

# TODO
