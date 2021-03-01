#pragma once

#include <dbus/dbus.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class AutopilotManagerConfig {
public:
	AutopilotManagerConfig() = default;
	~AutopilotManagerConfig() = default;

	bool AppendToMessage(DBusMessage *reply) const;
	bool InitFromMessage(DBusMessage *reply);
	bool WriteToFile(const std::string &config_path) const;
	bool InitFromFile(const std::string &config_path);

	bool autopilot_manager_enabled = false;
	std::string decision_maker_input_type = "";
	double simple_collision_avoid_distance_threshold = 0.0;
	std::string simple_collision_avoid_distance_on_condition_true = "";
	std::string simple_collision_avoid_distance_on_condition_false = "";
};
