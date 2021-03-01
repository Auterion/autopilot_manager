#pragma once

#include <functional>
#include <chrono>
#include <dbus/dbus.h>
#include <dbus/dbus-glib-lowlevel.h>
#include <iostream>
#include <thread>
#include <glib.h>

class DBusInterface {
public:
    struct Response {
        DBusMessage* request;
        std::string code;
    };

    using HandlerFunction = std::function<DBusMessage*(DBusMessage* request)>;

    DBusInterface(HandlerFunction function);
    ~DBusInterface();

    static constexpr auto BUS_NAME = "com.auterion.autopilot_manager";
    static constexpr auto INTERFACE_NAME = "com.auterion.autopilot_manager.interface";
    static constexpr auto OBJECT_NAME = "/com/auterion/autopilot_manager/interface";
private:
    DBusConnection* dbus_connection_;
    HandlerFunction handler_;
    const char *introspection =
	DBUS_INTROSPECT_1_0_XML_DOCTYPE_DECL_NODE
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
	"      <arg name='simple_collision_avoid_distance_threshold' type='d' direction='out' />\n"
	"      <arg name='simple_collision_avoid_distance_on_condition_true' type='s' direction='out' />\n"
	"      <arg name='simple_collision_avoid_distance_on_condition_false' type='s' direction='out' />\n"
	"      <arg name='response_code' type='u' direction='out' />\n"
	"    </method>\n"
	"    <method name='set_config'>\n"
	"      <arg name='autopilot_manager_enabled' type='u' direction='in' />\n"
	"      <arg name='decision_maker_input_type' type='s' direction='in' />\n"
	"      <arg name='simple_collision_avoid_distance_threshold' type='d' direction='in' />\n"
	"      <arg name='simple_collision_avoid_distance_on_condition_true' type='s' direction='in' />\n"
	"      <arg name='simple_collision_avoid_distance_on_condition_false' type='s' direction='in' />\n"
	"      <arg name='response_code' type='u' direction='out' />\n"
	"    </method>\n"
	"  </interface>\n"

	"</node>\n";

    void Register();
    static DBusHandlerResult MessageHandler(DBusConnection *conn, DBusMessage *message, void *data);
};
