cmake_minimum_required(VERSION 3.10.2)
project(autopilot-manager)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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
  src/modules/mission_manager
  src/modules/sensor_manager
  src/helpers
)

add_subdirectory(src/helpers)
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
    mission-manager
    sensor-manager
    helpers
    Threads::Threads
    ${GLIB_LDFLAGS}
    ${DBUS_LIBRARIES}
    ${LIBDBUS_LDFLAGS}
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
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(TARGETS autopilot-manager
  DESTINATION
  lib/${PROJECT_NAME}
)

# Install custom action JSON examples and dummy scripts
# Note: the default directory is specific to the AuterionOS
set(CUSTOM_ACTION_DATA_DIR "/usr/src/app/autopilot-manager/data/example/" CACHE FILEPATH "Path to custom action configuration examples and scripts")
# set(CUSTOM_ACTION_DATA_DIR "/usr/local/share/autopilot-manager/data/example/" CACHE FILEPATH "Path to custom action configuration examples and scripts")

set(CONFIG_DATA_DIR "/shared_container_dir/" CACHE FILEPATH "Path to autopilot manager configuration file")
# set(CUSTOM_ACTION_DATA_DIR "/usr/local/share/autopilot-manager/data/config/" CACHE FILEPATH "Path to autopilot manager configuration file")

# install(DIRECTORY data/example/
#   DESTINATION ${CUSTOM_ACTION_DATA_DIR})
#
# install(DIRECTORY data/config/
#   DESTINATION ${CONFIG_DATA_DIR})
#
# install(FILES com.auterion.autopilot_manager.conf
#   DESTINATION /usr/share/dbus-1/system.d/)

ament_package()
