cmake_minimum_required(VERSION 3.5)

project(mission_manager)

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
