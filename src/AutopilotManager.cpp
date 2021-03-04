#include <string>

#include <AutopilotManager.hpp>

namespace {
const auto METHOD_GET_CONFIG = "get_config";
const auto METHOD_SET_CONFIG = "set_config";
}  // namespace

AutopilotManager::AutopilotManager(const uint32_t& mavlinkPort, const std::string& configPath = "", const std::string& customActionConfigPath = "")
    : _mavlink_port(mavlinkPort),
      _config_path(configPath.empty() ? _config_path : configPath),
      _custom_action_config_path(customActionConfigPath.empty() ? _custom_action_config_path : customActionConfigPath)
{
	initialProvisioning();

    std::cout << "Autopilot Manager Enabled: " << std::boolalpha << _autopilot_manager_enabled << std::endl;

    start();
}

AutopilotManager::~AutopilotManager() {
    _mission_manager.reset();
}

DBusMessage* AutopilotManager::HandleRequest(DBusMessage* message) {
	DBusMessage* reply = nullptr;
	if (dbus_message_is_method_call(message, DBusInterface::INTERFACE_NAME, METHOD_GET_CONFIG)) {
		std::cout << "[Autopilot Manager] Received message: " << METHOD_GET_CONFIG << std::endl;
		if (!(reply = dbus_message_new_method_return(message))) {
			std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return"
				  << std::endl;
		} else {
			AutopilotManagerConfig config;
			config.InitFromFile(_config_path);
			ResponseCode response_code = GetConfiguration(config);
			config.AppendToMessage(reply);
			dbus_message_append_args(reply, DBUS_TYPE_UINT32, &response_code, DBUS_TYPE_INVALID);
		}
	} else if (dbus_message_is_method_call(message, DBusInterface::INTERFACE_NAME, METHOD_SET_CONFIG)) {
		std::cout << "[Autopilot Manager] Received message: " << METHOD_SET_CONFIG << std::endl;
		if (!(reply = dbus_message_new_method_return(message))) {
			std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return"
				  << std::endl;
		} else {
			AutopilotManagerConfig config;
			ResponseCode response_code;
			if (config.InitFromMessage(message)) {
				response_code = SetConfiguration(config);
				config.WriteToFile(_config_path);
			} else {
				response_code = ResponseCode::FAILED;
			}
			config.AppendToMessage(reply);
			dbus_message_append_args(reply, DBUS_TYPE_UINT32, &response_code, DBUS_TYPE_INVALID);
		}
	}
	return reply;
}

void AutopilotManager::initialProvisioning() {
	AutopilotManagerConfig config;
	if (config.InitFromFile(_config_path)) {
		ResponseCode response_code = SetConfiguration(config);
		std::cout << "[Autopilot Manager] Initial provisioning finished with code " << response_code
			  << std::endl;
	} else {
		std::cout << "[Autopilot Manager] Failed to init default config from " << _config_path << std::endl;
	}
}

AutopilotManager::ResponseCode AutopilotManager::SetConfiguration(AutopilotManagerConfig& config) {
	_autopilot_manager_enabled = config.autopilot_manager_enabled;
	_decision_maker_input_type = config.decision_maker_input_type;
    _simple_collision_avoid_enabled = config.simple_collision_avoid_enabled;
	_simple_collision_avoid_distance_threshold = config.simple_collision_avoid_distance_threshold;
	_simple_collision_avoid_distance_on_condition_true = config.simple_collision_avoid_distance_on_condition_true;
	_simple_collision_avoid_distance_on_condition_false = config.simple_collision_avoid_distance_on_condition_false;

    if (!config.simple_collision_avoid_enabled) {
        return ResponseCode::SUCCEED_WITH_COLL_AVOID_OFF;
    } else {
        return ResponseCode::SUCCEED_WITH_COLL_AVOID_ON;
    }

	return ResponseCode::UNKNOWN;
}

AutopilotManager::ResponseCode AutopilotManager::GetConfiguration(AutopilotManagerConfig& config) {
	config.autopilot_manager_enabled = _autopilot_manager_enabled;
	config.decision_maker_input_type = _decision_maker_input_type;
    config.simple_collision_avoid_enabled = _simple_collision_avoid_enabled;
	config.simple_collision_avoid_distance_threshold = _simple_collision_avoid_distance_threshold;
	config.simple_collision_avoid_distance_on_condition_true = _simple_collision_avoid_distance_on_condition_true;
	config.simple_collision_avoid_distance_on_condition_false = _simple_collision_avoid_distance_on_condition_false;

    if (!config.simple_collision_avoid_enabled) {
        return ResponseCode::SUCCEED_WITH_COLL_AVOID_OFF;
    } else {
        return ResponseCode::SUCCEED_WITH_COLL_AVOID_ON;
    }

	return ResponseCode::UNKNOWN;
}

int AutopilotManager::start() {
    // Configure MAVSDK Mission Manager instance
	mavsdk::Mavsdk mavsdk_mission_computer;

	// Change configuration so the instance is treated as a mission computer
	mavsdk::Mavsdk::Configuration config_cc(mavsdk::Mavsdk::Configuration::UsageType::CompanionComputer);
	mavsdk_mission_computer.set_configuration(config_cc);

	auto system = std::shared_ptr<mavsdk::System>{nullptr};

	mavsdk::ConnectionResult ret_comp = mavsdk_mission_computer.add_udp_connection(_mavlink_port);
	if (ret_comp == mavsdk::ConnectionResult::Success) {
		std::cout << "[AutopilotManagerMain] Waiting to discover vehicle from the mission computer side..."
			  << std::endl;
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
        auto fut = prom.get_future();

        // We wait for new systems to be discovered, once we find one that has an
        // autopilot, we decide to use it
		mavsdk_mission_computer.subscribe_on_new_system([&prom, &mavsdk_mission_computer]() {
            auto system = mavsdk_mission_computer.systems().back();

			if (system->has_autopilot()) {
                std::cout << "Discovered autopilot\n";

                // Unsubscribe again as we only want to find one system.
                mavsdk_mission_computer.subscribe_on_new_system(nullptr);
				prom.set_value(system);
			}

		});

        // We usually receive heartbeats at 1Hz, therefore we should find a
        // system after around 3 seconds max
        if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
            std::cerr << "No autopilot found, exiting.\n";
            return 1;
        }

        // Get discovered system now
        auto system = fut.get();

		// Start the Mission Manager module
		_mission_manager = std::make_shared<MissionManager>(system, _custom_action_config_path);

	} else {
		std::cerr << "[AutopilotManagerMain] Failed to connect to port " << _mavlink_port << std::endl;
        return 1;
	}

    return 0;
}
