cmake_minimum_required(VERSION 3.5)
project(mission_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(MAVSDK REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(landing_mapper REQUIRED)
find_package(landing_planner REQUIRED)
find_package(timing_tools REQUIRED)

add_library(
    mission-manager SHARED
    MissionManager.cpp
    CustomActionHandler.cpp
)
ament_target_dependencies(mission-manager
  timing_tools
)
target_link_libraries(mission-manager
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_custom_action
    MAVSDK::mavsdk_mission_raw
    MAVSDK::mavsdk_param
    MAVSDK::mavsdk_server_utility
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mavlink_passthrough
    MAVSDK::mavsdk
    Eigen3::Eigen
    ${landing_mapper_LIBRARIES}
    ${landing_planner_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

target_include_directories(mission-manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/autopilot_manager/mission_manager>
    ${Eigen3_INCLUDE_DIRS}
    ${landing_planner_INCLUDE_DIRS}
    ${landing_mapper_INCLUDE_DIRS}
)

install(TARGETS mission-manager
    EXPORT mission-manager
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
