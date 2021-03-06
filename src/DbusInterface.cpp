#include <DbusInterface.hpp>

DBusInterface::DBusInterface(HandlerFunction handler) : handler_(handler) {
	DBusError dbus_error;

	dbus_error_init(&dbus_error);

	dbus_connection_ = dbus_bus_get(DBUS_BUS_SYSTEM, &dbus_error);
	if (dbus_error_is_set(&dbus_error)) {
		std::cerr << "[DBusInterface] dbus_bus_get:" << dbus_error.message << std::endl;
		dbus_error_free(&dbus_error);
	}

	if (!dbus_connection_) {
		throw std::runtime_error("[DbusInterface] Unable to get Dbus connection");
	}
	const DBusObjectPathVTable server_vtable = {.unregister_function = NULL,
						    .message_function = &DBusInterface::MessageHandler,
						    .dbus_internal_pad1 = NULL,
						    .dbus_internal_pad2 = NULL,
						    .dbus_internal_pad3 = NULL,
						    .dbus_internal_pad4 = NULL};
	if (!dbus_connection_register_object_path(dbus_connection_, OBJECT_NAME, &server_vtable, this)) {
		std::cerr << std::endl
			  << "[DBusInterface] dbus_connection_register_object_path: " << dbus_error.message
			  << std::endl;
		throw std::runtime_error("Failed to register the object to dbus");
	}

	Register();
	dbus_connection_setup_with_g_main(dbus_connection_, NULL);
	std::cout << "[DbusInterface] Service registered to dbus" << std::endl;
}

void DBusInterface::Register() {
	DBusError dbus_error;

	dbus_error_init(&dbus_error);

	while (true) {
		const int return_code =
		    dbus_bus_request_name(dbus_connection_, DBusInterface::BUS_NAME, 0, &dbus_error);

		if (return_code == DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER) {
			return;
		} else if (return_code == DBUS_REQUEST_NAME_REPLY_IN_QUEUE) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		if (dbus_error_is_set(&dbus_error)) {
			std::cerr << std::endl
				  << "[DBusInterface] dbus_bus_request_name: " << dbus_error.message << std::endl;
			dbus_error_free(&dbus_error);
			throw std::runtime_error("Failed to register the service on dbus");
		}
	}
}

DBusInterface::~DBusInterface() {}

DBusHandlerResult DBusInterface::MessageHandler(DBusConnection *conn, DBusMessage *message, void *data) {
	std::cout << "[DBusInterface] Received message: " << dbus_message_get_interface(message) << " "
		  << dbus_message_get_member(message) << " " << dbus_message_get_path(message) << std::endl;

	// DBusHandlerResult result;
	DBusMessage *reply = nullptr;
	DBusInterface *interface = (DBusInterface *)data;

	if (dbus_message_is_method_call(message, "org.freedesktop.DBus.Introspectable", "Introspect")) {
		if (!(reply = dbus_message_new_method_return(message))) {
			std::cerr << "[DBusInterface] Error: dbus_message_new_method_return" << std::endl;
		} else {
			dbus_message_append_args(reply, DBUS_TYPE_STRING, &interface->introspection, DBUS_TYPE_INVALID);
		}
	} else if (dbus_message_is_method_call(message, "org.freedesktop.DBus.Peer", "Ping")) {
		if (!(reply = dbus_message_new_method_return(message))) {
			std::cerr << "[DBusInterface] Error: dbus_message_new_method_return" << std::endl;
		}
	} else {
		reply = interface->handler_(message);
	}

	// dbus_message_unref(message);
	if (reply != nullptr) {
		if (!dbus_connection_send(conn, reply, NULL)) {
			std::cerr << "[DBusInterface] Error: dbus_connection_send" << std::endl;
		}
		dbus_message_unref(reply);
		return DBUS_HANDLER_RESULT_HANDLED;
	}
	return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}
