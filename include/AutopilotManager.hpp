#pragma once

#include <dbus/dbus.h>
#include <DbusInterface.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include <mavsdk/mavsdk.h>

#include <AutopilotManagerConfig.hpp>
#include <MissionManager.hpp>

class AutopilotManager {
public:
	AutopilotManager(const uint32_t& mavlinkPort, const std::string& configPath, const std::string& customActionConfigPath);
	~AutopilotManager();
	DBusMessage* HandleRequest(DBusMessage* request);

	void setAutopilotManagerEnabled(const bool& enable) { _autopilot_manager_enabled = enable; }
	bool autopilotManagerEnabled() { return _autopilot_manager_enabled; }

	void setDecisionMakerInputType(const std::string& decision_maker_input) { _decision_maker_input_type = decision_maker_input; }
	std::string decisionMakerInputType() { return _decision_maker_input_type; }

	void setSimpleCollAvoidEnabled(const bool& enable) { _simple_collision_avoid_enabled = enable; }
	bool simpleCollAvoidEnabled() { return _simple_collision_avoid_enabled; }

	void setSimpleCollAvoidDistanceThreshold(const double& thr) { _simple_collision_avoid_distance_threshold = thr; }
	double simpleCollAvoidDistanceThreshold() { return _simple_collision_avoid_distance_threshold; }

	void setSimpleCollAvoidOnConditionTrueAction(const std::string& action) { _simple_collision_avoid_distance_on_condition_true = action; }
	std::string simpleCollAvoidOnConditionTrueAction() { return _simple_collision_avoid_distance_on_condition_true; }

	void setSimpleCollAvoidOnConditionFalseAction(const std::string& action) { _simple_collision_avoid_distance_on_condition_false = action; }
	std::string simpleCollAvoidOnConditionFalseAction() { return _simple_collision_avoid_distance_on_condition_false; }

private:
	enum ResponseCode {
		SUCCEED_WITH_COLL_AVOID_OFF = 0,
		SUCCEED_WITH_COLL_AVOID_ON = 1,
		FAILED = 2,
		UNKNOWN = 999
	};

	int start();
	void initialProvisioning();

	ResponseCode SetConfiguration(AutopilotManagerConfig& config);
	ResponseCode GetConfiguration(AutopilotManagerConfig& config);

	bool _autopilot_manager_enabled = false;
	std::string _decision_maker_input_type = "";
	bool _simple_collision_avoid_enabled = false;
	double _simple_collision_avoid_distance_threshold = 0.0;
	std::string _simple_collision_avoid_distance_on_condition_true = "";
	std::string _simple_collision_avoid_distance_on_condition_false = "";

	std::shared_ptr<MissionManager> _mission_manager = {nullptr};

	uint32_t _mavlink_port = 0;
	std::string _config_path = "/shared_container_dir/autopilot_manager.conf";
	std::string _custom_action_config_path = "/usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json";
};
