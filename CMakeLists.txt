cmake_minimum_required(VERSION 3.10.2)
project(autopilot-manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_search_module(DBUS REQUIRED dbus-1)
pkg_search_module(LIBDBUS REQUIRED dbus-glib-1)

find_package(image_downsampler REQUIRED)
if(NOT MSVC)
    add_definitions("-Wall -Wextra ")
    # This warning is triggered by the MAVLink includes.
    add_definitions("-Wno-address-of-packed-member")
else()
    add_definitions("-WX -W2")
endif()

include_directories(
  include
  src/modules
  src/modules/collision_avoidance_manager
  src/modules/landing_manager
  src/modules/mission_manager
  src/modules/sensor_manager
  src/helpers
)

add_subdirectory(src/helpers)
add_subdirectory(src/modules/collision_avoidance_manager)
add_subdirectory(src/modules/landing_manager)
add_subdirectory(src/modules/mission_manager)
add_subdirectory(src/modules/sensor_manager)

add_executable(
    autopilot-manager
    src/main.cpp
    src/AutopilotManager.cpp
    src/AutopilotManagerConfig.cpp
    src/DbusInterface.cpp
)

target_link_libraries(
    autopilot-manager
    collision-avoidance-manager
    landing-manager
    mission-manager
    sensor-manager
    helpers
    Threads::Threads
    ${GLIB_LDFLAGS}
    ${DBUS_LIBRARIES}
    ${LIBDBUS_LDFLAGS}
    tbb
)

target_include_directories(autopilot-manager
    PRIVATE
    include
    ${GLIB_INCLUDE_DIRS}
    ${DBUS_INCLUDE_DIRS}
    ${LIBDBUS_INCLUDE_DIRS}
)

# Install autopilot-manager
install(
  TARGETS autopilot-manager
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
# To run with 'ros2 run':
install(TARGETS autopilot-manager
  DESTINATION
  lib/${PROJECT_NAME}
)

set(CUSTOM_ACTION_DATA_DIR "share/${PROJECT_NAME}/data/example/" CACHE FILEPATH "Path to custom action configuration examples and scripts")
set(CONFIG_DATA_DIR "share/${PROJECT_NAME}/data/config/" CACHE FILEPATH "Path to autopilot manager default parameters file")
set(DBUS_CONFIG_DIR "share/${PROJECT_NAME}/dbus/" CACHE FILEPATH "Path to autopilot manager DBUS configuration file")

# ## Note for usage in the Skynode: ##
# As the Autopilot Manager will be installed in a read-only partition,
# this file should be copied/moved manually to /shared_container_dir/
# if there is a need to edit it. Otherwise, one can create a new JSON
# file and load it using the provided '-a' option
install(DIRECTORY data/example/
  DESTINATION ${CUSTOM_ACTION_DATA_DIR})

# ## Note for usage in the Skynode: ##
# As the Autopilot Manager will be installed in a read-only partition,
# this file should be copied/moved manually to /shared_container_dir/.
# A different file can be used and loaded through the '-c' option
install(DIRECTORY data/config/
  DESTINATION ${CONFIG_DATA_DIR})

# This should be copied/moved manually to /usr/share/dbus-1/system.d/
install(FILES com.auterion.autopilot_manager.conf
  DESTINATION ${DBUS_CONFIG_DIR})

ament_package()
