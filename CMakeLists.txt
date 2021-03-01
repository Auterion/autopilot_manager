cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(autopilot_manager)

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

include_directories(include)

add_subdirectory(src/helpers)
add_subdirectory(src/modules)

add_executable(
    autopilot-manager
    src/main.cpp
    src/AutopilotManager.cpp
    src/AutopilotManagerConfig.cpp
    src/DbusInterface.cpp
)

target_link_libraries(
    autopilot-manager
    PUBLIC
    mission-manager
    helpers
    PRIVATE
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
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

# Install custom action JSON examples and dummy scripts
# Note: the default directory is specific to the AuterionOS
set(CUSTOM_ACTION_DATA_DIR "/usr/src/app/autopilot-manager/data" CACHE FILEPATH "Path to custom action JSON examples and scripts")
# set(CUSTOM_ACTION_DATA_DIR "/usr/local/share/autopilot-manager/data" CACHE FILEPATH "Path to custom action JSON examples and scripts")

install(DIRECTORY data/
  DESTINATION ${CUSTOM_ACTION_DATA_DIR})

install(FILES com.auterion.autopilot_manager.conf
  DESTINATION /usr/share/dbus-1/system.d/)
