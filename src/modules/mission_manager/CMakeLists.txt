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

add_library(
    mission-manager
    MissionManager.cpp
)

target_link_libraries(
    mission-manager
    PUBLIC
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_custom_action
    MAVSDK::mavsdk
    PRIVATE Threads::Threads
)

target_include_directories(mission-manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/autopilot_manager/mission_manager>
    )

install(TARGETS mission-manager
    EXPORT mission-manager
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
