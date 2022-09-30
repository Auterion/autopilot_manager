#pragma once

#include <dbus/dbus-glib-lowlevel.h>
#include <dbus/dbus.h>
#include <glib.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

class DBusInterface {
   public:
    struct Response {
        DBusMessage *request;
        std::string code;
    };

    using HandlerFunction = std::function<DBusMessage *(DBusMessage *request)>;

    explicit DBusInterface(HandlerFunction handler);
    ~DBusInterface();

    static constexpr auto BUS_NAME = "com.auterion.autopilot_manager";
    static constexpr auto INTERFACE_NAME = "com.auterion.autopilot_manager.interface";
    static constexpr auto OBJECT_NAME = "/com/auterion/autopilot_manager/interface";

   private:
    DBusConnection *dbus_connection_;
    HandlerFunction handler_;
    const char *introspection = DBUS_INTROSPECT_1_0_XML_DOCTYPE_DECL_NODE
        "<node>\n"
        "  <interface name='org.freedesktop.DBus.Introspectable'>\n"
        "    <method name='Introspect'>\n"
        "      <arg name='data' type='s' direction='out' />\n"
        "    </method>\n"
        "  </interface>\n"

        "  <interface name='org.freedesktop.DBus.Peer'>\n"
        "    <method name='Ping'/>\n"
        "  </interface>\n"

        "  <interface name='com.auterion.autopilot_manager.interface'>\n"
        "    <method name='get_config'>\n"
        "      <arg name='autopilot_manager_enabled' type='u' direction='out' />\n"
        "      <arg name='decision_maker_input_type' type='s' direction='out' />\n"
        "      <arg name='script_to_call' type='s' direction='out' />\n"
        "      <arg name='api_call' type='s' direction='out' />\n"
        "      <arg name='local_position_offset_x' type='d' direction='out' />\n"
        "      <arg name='local_position_offset_y' type='d' direction='out' />\n"
        "      <arg name='local_position_offset_z' type='d' direction='out' />\n"
        "      <arg name='local_position_waypoint_x' type='d' direction='out' />\n"
        "      <arg name='local_position_waypoint_y' type='d' direction='out' />\n"
        "      <arg name='local_position_waypoint_z' type='d' direction='out' />\n"
        "      <arg name='global_position_offset_lat' type='d' direction='out' />\n"
        "      <arg name='global_position_offset_lon' type='d' direction='out' />\n"
        "      <arg name='global_position_offset_alt_amsl' type='d' direction='out' />\n"
        "      <arg name='global_position_waypoint_lat' type='d' direction='out' />\n"
        "      <arg name='global_position_waypoint_lon' type='d' direction='out' />\n"
        "      <arg name='global_position_waypoint_alt_amsl' type='d' direction='out' />\n"
        "      <arg name='safe_landing_enabled' type='u' direction='out' />\n"
        "      <arg name='safe_landing_area_square_size' type='d' direction='out' />\n"
        "      <arg name='safe_landing_distance_to_ground' type='d' direction='out' />\n"
        "      <arg name='safe_landing_on_no_safe_land' type='s' direction='out' />\n"
        "      <arg name='safe_landing_try_landing_after_action' type='u' direction='out' />\n"
        "      <arg name='landing_site_search_max_distance' type='d' direction='out' />\n"
        "      <arg name='landing_site_search_min_height' type='d' direction='out' />\n"
        "      <arg name='landing_site_search_spiral_spacing' type='d' direction='out' />\n"
        "      <arg name='simple_collision_avoid_enabled' type='u' direction='out' />\n"
        "      <arg name='simple_collision_avoid_distance_threshold' type='d' direction='out' />\n"
        "      <arg name='simple_collision_avoid_action_on_condition_true' type='s' direction='out' />\n"
        "      <arg name='response_code' type='u' direction='out' />\n"
        "    </method>\n"
        "    <method name='set_config'>\n"
        "      <arg name='autopilot_manager_enabled' type='u' direction='in' />\n"
        "      <arg name='decision_maker_input_type' type='s' direction='in' />\n"
        "      <arg name='script_to_call' type='s' direction='in' />\n"
        "      <arg name='api_call' type='s' direction='in' />\n"
        "      <arg name='local_position_offset_x' type='d' direction='in' />\n"
        "      <arg name='local_position_offset_y' type='d' direction='in' />\n"
        "      <arg name='local_position_offset_z' type='d' direction='in' />\n"
        "      <arg name='local_position_waypoint_x' type='d' direction='in' />\n"
        "      <arg name='local_position_waypoint_y' type='d' direction='in' />\n"
        "      <arg name='local_position_waypoint_z' type='d' direction='in' />\n"
        "      <arg name='global_position_offset_lat' type='d' direction='in' />\n"
        "      <arg name='global_position_offset_lon' type='d' direction='in' />\n"
        "      <arg name='global_position_offset_alt_amsl' type='d' direction='in' />\n"
        "      <arg name='global_position_waypoint_lat' type='d' direction='in' />\n"
        "      <arg name='global_position_waypoint_lon' type='d' direction='in' />\n"
        "      <arg name='global_position_waypoint_alt_amsl' type='d' direction='in' />\n"
        "      <arg name='safe_landing_enabled' type='u' direction='in' />\n"
        "      <arg name='safe_landing_area_square_size' type='d' direction='in' />\n"
        "      <arg name='safe_landing_distance_to_ground' type='d' direction='in' />\n"
        "      <arg name='safe_landing_on_no_safe_land' type='s' direction='in' />\n"
        "      <arg name='safe_landing_try_landing_after_action' type='u' direction='in' />\n"
        "      <arg name='landing_site_search_max_distance' type='d' direction='in' />\n"
        "      <arg name='landing_site_search_min_height' type='d' direction='in' />\n"
        "      <arg name='landing_site_search_spiral_spacing' type='d' direction='in' />\n"
        "      <arg name='simple_collision_avoid_enabled' type='u' direction='in' />\n"
        "      <arg name='simple_collision_avoid_distance_threshold' type='d' direction='in' />\n"
        "      <arg name='simple_collision_avoid_action_on_condition_true' type='s' direction='in' />\n"
        "      <arg name='response_code' type='u' direction='out' />\n"
        "    </method>\n"
        "  </interface>\n"

        "</node>\n";

    void Register();
    static DBusHandlerResult MessageHandler(DBusConnection *conn, DBusMessage *message, void *data);
};
