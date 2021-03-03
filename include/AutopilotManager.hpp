#pragma once

#include <dbus/dbus.h>
#include <DbusInterface.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include <AutopilotManagerConfig.hpp>

class AutopilotManager {
public:
	AutopilotManager(const std::string& configPath);
	~AutopilotManager() = default;
	DBusMessage* HandleRequest(DBusMessage* request);

	void setAutopilotManagerEnabled(bool enable) { _autopilot_manager_enabled = enable; }

	bool autopilotManagerEnabled() { return _autopilot_manager_enabled; }

	void setAutopilotFound() { _autopilot_found = true; }

	bool autopilotFound() { return _autopilot_found; }

private:
	enum ResponseCode {
		SUCCEED_WITH_COLL_AVOID_OFF = 0,
		SUCCEED_WITH_COLL_AVOID_ON = 1,
		FAILED = 2,
		UNKNOWN = 999
	};

	void initialProvisioning();
	ResponseCode SetConfiguration(AutopilotManagerConfig& config);
	ResponseCode GetConfiguration(AutopilotManagerConfig& config);

	bool _autopilot_manager_enabled = false;
	std::string _decision_maker_input_type = "";
	bool _simple_collision_avoid_enabled = false;
	double _simple_collision_avoid_distance_threshold = 0.0;
	std::string _simple_collision_avoid_distance_on_condition_true = "";
	std::string _simple_collision_avoid_distance_on_condition_false = "";

	const std::string _configPath = "/shared_container_dir/autopilot_manager.conf";

	bool _autopilot_found{false};
};
