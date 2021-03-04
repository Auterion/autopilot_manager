#include <AutopilotManagerConfig.hpp>

bool AutopilotManagerConfig::InitFromMessage(DBusMessage* request) {
	if (request != nullptr) {
		DBusError error;
		dbus_error_init(&error);
		bool* returnedEnabled = nullptr;
		char* returnedDecisionMakerType = nullptr;
		bool* returnedCollAvoidEnabled = nullptr;
		double* returnedCollAvoidThr = nullptr;
		char* returnedCollAvoidOnCondTrue = nullptr;
		char* returnedCollAvoidOnCondFalse = nullptr;
		dbus_message_get_args(request, &error, DBUS_TYPE_BOOLEAN, &returnedEnabled, DBUS_TYPE_STRING,
				      &returnedDecisionMakerType, DBUS_TYPE_DOUBLE, &returnedCollAvoidThr,
				      DBUS_TYPE_STRING, &returnedCollAvoidOnCondTrue, DBUS_TYPE_STRING,
				      &returnedCollAvoidOnCondFalse, DBUS_TYPE_INVALID);
		if (dbus_error_is_set(&error)) {
			std::cerr << "[Autopilot Manager Config] Failed getting message arguments: " << error.message
				  << std::endl;
			dbus_error_free(&error);
		} else {
			autopilot_manager_enabled = (*returnedEnabled);
			decision_maker_input_type = (*returnedDecisionMakerType);
			simple_collision_avoid_enabled = (*returnedCollAvoidEnabled);
			simple_collision_avoid_distance_threshold = (*returnedCollAvoidThr);
			simple_collision_avoid_distance_on_condition_true = (*returnedCollAvoidOnCondTrue);
			simple_collision_avoid_distance_on_condition_false = (*returnedCollAvoidOnCondFalse);

			std::cout << "[AutopilotManagerConfig] Received: [" << std::endl;
			std::cout << "    autopilot_manager_enabled: " << autopilot_manager_enabled << std::endl;
			std::cout << "    decision_maker_input_type: " << decision_maker_input_type << std::endl;
			std::cout << "    simple_collision_avoid_enabled: " << simple_collision_avoid_enabled
				  << std::endl;
			std::cout << "    simple_collision_avoid_distance_threshold: "
				  << simple_collision_avoid_distance_threshold << std::endl;
			std::cout << "    simple_collision_avoid_distance_on_condition_true: "
				  << simple_collision_avoid_distance_on_condition_true << std::endl;
			std::cout << "    simple_collision_avoid_distance_on_condition_false: "
				  << simple_collision_avoid_distance_on_condition_false << std::endl;
			std::cout << "]" << std::endl;
			return true;
		}
	}
	return false;
}

bool AutopilotManagerConfig::AppendToMessage(DBusMessage* reply) const {
	if (reply != nullptr) {
		dbus_message_append_args(reply, DBUS_TYPE_BOOLEAN, &autopilot_manager_enabled, DBUS_TYPE_STRING,
					 &decision_maker_input_type, DBUS_TYPE_DOUBLE, DBUS_TYPE_BOOLEAN,
					 &simple_collision_avoid_enabled,
					 &simple_collision_avoid_distance_threshold, DBUS_TYPE_STRING,
					 &simple_collision_avoid_distance_on_condition_true, DBUS_TYPE_STRING,
					 &simple_collision_avoid_distance_on_condition_false, DBUS_TYPE_INVALID);
		return true;
	}
	return false;
}

bool AutopilotManagerConfig::WriteToFile(const std::string& config_path) const {
	const std::string temp_path = config_path + ".tmp";
	std::ofstream file(temp_path);
	if (file.is_open()) {
		file << "[AutopilotManagerConfig]" << std::endl;
		file << "autopilot_manager_enabled=" << std::to_string(autopilot_manager_enabled) << std::endl;
		file << "decision_maker_input_type=" << decision_maker_input_type << std::endl;
		file << "simple_collision_avoid_enabled=" << std::to_string(simple_collision_avoid_enabled) << std::endl;
		file << "simple_collision_avoid_distance_threshold="
		     << std::to_string(simple_collision_avoid_distance_threshold) << std::endl;
		file << "simple_collision_avoid_distance_on_condition_true="
		     << simple_collision_avoid_distance_on_condition_true << std::endl;
		file << "simple_collision_avoid_distance_on_condition_false="
		     << simple_collision_avoid_distance_on_condition_false << std::endl;
		file.close();
		if (rename(temp_path.c_str(), config_path.c_str()) != 0) {
			std::cerr << "[Autopilot Manager Config] Unable to write file to: " << config_path << std::endl;
			return false;
		}
		return true;
	} else {
		std::cerr << "[Autopilot Manager Config] Unable to open file: " << config_path << std::endl;
		return false;
	}
}

bool AutopilotManagerConfig::InitFromFile(const std::string& config_path) {
	std::ifstream file(config_path);
	if (file.is_open()) {
		std::istringstream sin;
		std::string line;
		std::cout << "[Autopilot Manager Config] Loaded config from file:" << std::endl;
		while (std::getline(file, line)) {
			sin.str(line.substr(line.find("=") + 1));
			if (line.find("autopilot_manager_enabled") != std::string::npos) {
				std::cout << "\tautopilot_manager_enabled: " << sin.str() << std::endl;
				autopilot_manager_enabled = (sin.str() == "true");
			}
			if (line.find("decision_maker_input_type") != std::string::npos) {
				std::cout << "\tdecision_maker_input_type: " << sin.str() << std::endl;
				decision_maker_input_type = sin.str();
			}
			if (line.find("simple_collision_avoid_enabled") != std::string::npos) {
				std::cout << "\tsimple_collision_avoid_enabled: " << sin.str() << std::endl;
				simple_collision_avoid_enabled = (sin.str() == "true");
			}
			if (line.find("simple_collision_avoid_distance_threshold") != std::string::npos) {
				std::cout << "\tsimple_collision_avoid_distance_threshold: " << sin.str() << std::endl;
				simple_collision_avoid_distance_threshold = std::stod(sin.str());
			}
			if (line.find("simple_collision_avoid_distance_on_condition_true") != std::string::npos) {
				std::cout << "\tsimple_collision_avoid_distance_on_condition_true: " << sin.str()
					  << std::endl;
				simple_collision_avoid_distance_on_condition_true = sin.str();
			}
			if (line.find("simple_collision_avoid_distance_on_condition_false") != std::string::npos) {
				std::cout << "\tsimple_collision_avoid_distance_on_condition_false: " << sin.str()
					  << std::endl;
				simple_collision_avoid_distance_on_condition_false = sin.str();
			}
			sin.clear();
		}
		file.close();
		return true;
	} else {
		std::cerr << "[AutopilotManagerConfig] Unable to open file: " << config_path << std::endl;
		return false;
	}
}
