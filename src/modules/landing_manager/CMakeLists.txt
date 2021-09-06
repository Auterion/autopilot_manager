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


#################
# Setup targets #
#################

add_library(landing-manager SHARED LandingManager.cpp)
ament_target_dependencies(landing-manager Eigen3)
target_include_directories(landing-manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/autopilot-manager/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(landing-manager
  Eigen3::Eigen
  ${rclcpp_LIBRARIES}
)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3)
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
