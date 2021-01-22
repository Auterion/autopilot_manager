cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(autopilot_manager)

if(NOT MSVC)
    add_definitions("-Wall -Wextra ")
    # This warning is triggered by the MAVLink includes.
    add_definitions("-Wno-address-of-packed-member")
else()
    add_definitions("-WX -W2")
endif()

add_subdirectory(src/helpers)
add_subdirectory(src/modules)

add_executable(
    autopilot-manager
    src/main.cpp
)

target_link_libraries(
    autopilot-manager
    mission-manager
    helpers
)

# Install autopilot-manager
install(TARGETS autopilot-manager RUNTIME DESTINATION bin)

# Install custom action JSON examples and dummy scripts
# Note: the default directory is specific to the AuterionOS
# set(CUSTOM_ACTION_DATA_DIR "/usr/src/app/autopilot-manager/data" CACHE FILEPATH "Path to custom action JSON examples and scripts")
set(CUSTOM_ACTION_DATA_DIR "share/autopilot-manager/data" CACHE FILEPATH "Path to custom action JSON examples and scripts")
install(DIRECTORY data/ DESTINATION ${CUSTOM_ACTION_DATA_DIR})
