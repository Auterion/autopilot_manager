#pragma once

#include <dbus/dbus.h>
#include <DbusInterface.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include <mavsdk/mavsdk.h>

#include <AutopilotManagerConfig.hpp>
#include <mission_manager/MissionManager.hpp>
#include <sensor_manager/SensorManager.hpp>

class AutopilotManager {
public:
	AutopilotManager(const std::string& mavlinkPort, const std::string& configPath,
			 const std::string& customActionConfigPath);
	~AutopilotManager();
	DBusMessage* HandleRequest(DBusMessage* request);

private:
	enum ResponseCode {
		SUCCEED_WITH_COLL_AVOID_OFF = 0,
		SUCCEED_WITH_COLL_AVOID_ON = 1,
		FAILED = 2,
		UNKNOWN = 999
	};

	void start();
	void initialProvisioning();

	void init_mission_manager();
	void init_and_run_sensor_manager();

	ResponseCode SetConfiguration(AutopilotManagerConfig& config);
	ResponseCode GetConfiguration(AutopilotManagerConfig& config);

	uint8_t _autopilot_manager_enabled = false;
	std::string _decision_maker_input_type = "";
	uint8_t _simple_collision_avoid_enabled = false;
	double _simple_collision_avoid_distance_threshold = 0.0;
	std::string _simple_collision_avoid_distance_on_condition_true = "";
	std::string _simple_collision_avoid_distance_on_condition_false = "";

	std::thread _mission_manager_th;
	std::thread _sensor_manager_th;

	std::shared_ptr<MissionManager> _mission_manager = {nullptr};
	std::shared_ptr<SensorManager> _sensor_manager = {nullptr};

	std::mutex _config_mutex;

	std::string _mavlink_port = "14590";
	std::string _config_path = "/shared_container_dir/autopilot_manager.conf";
	std::string _custom_action_config_path =
	    "/usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json";
};
