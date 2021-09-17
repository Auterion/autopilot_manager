cmake_minimum_required(VERSION 3.5)
project(mission_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(
    mission-manager SHARED
    MissionManager.cpp
    CustomActionHandler.cpp
)

target_link_libraries(
    mission-manager
    PUBLIC
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_custom_action
    MAVSDK::mavsdk_server_utility
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    PRIVATE Threads::Threads
    Eigen3::Eigen
)

target_include_directories(mission-manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/autopilot_manager/mission_manager>
    ${Eigen3_INCLUDE_DIRS}
    )

install(TARGETS mission-manager
    EXPORT mission-manager
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
