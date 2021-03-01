#include <string>

#include <AutopilotManagerConfig.hpp>
#include <AutopilotManager.hpp>

namespace {
    const auto METHOD_GET_CONFIG = "get_config";
    const auto METHOD_SET_CONFIG = "set_config";
}

AutopilotManager::AutopilotManager(const std::string& configPath = "") :
    _configPath(configPath.empty() ? _configPath : configPath)
{
    initialProvisioning();
}

DBusMessage* AutopilotManager::HandleRequest(DBusMessage* message) {
    DBusMessage *reply = nullptr;
    if (dbus_message_is_method_call(message, DBusInterface::INTERFACE_NAME, METHOD_GET_CONFIG)) {
        std::cout << "[Autopilot Manager] Received message: " << METHOD_GET_CONFIG << std::endl;
        if (!(reply = dbus_message_new_method_return(message))) {
            std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return" << std::endl;
        } else {
            AutopilotManagerConfig config;
            config.InitFromFile(_configPath);
            ResponseCode response_code = GetConfiguration(config);
            config.AppendToMessage(reply);
            dbus_message_append_args(reply,
                    DBUS_TYPE_UINT32, &response_code,
                    DBUS_TYPE_INVALID);
        }
    } else if (dbus_message_is_method_call(message, DBusInterface::INTERFACE_NAME, METHOD_SET_CONFIG)) {
        std::cout << "[Autopilot Manager] Received message: " << METHOD_SET_CONFIG << std::endl;
        if (!(reply = dbus_message_new_method_return(message))) {
            std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return" << std::endl;
        } else {
            AutopilotManagerConfig config;
            ResponseCode response_code;
            if(config.InitFromMessage(message)) {
                response_code = SetConfiguration(config);
                config.WriteToFile(_configPath);
            } else {
                response_code = ResponseCode::FAILED;
            }
            config.AppendToMessage(reply);
            dbus_message_append_args(reply,
                    DBUS_TYPE_UINT32, &response_code,
                    DBUS_TYPE_INVALID);
        }
    }
    return reply;
}

void AutopilotManager::initialProvisioning() {
    AutopilotManagerConfig config;
    if(config.InitFromFile(_configPath)) {
        ResponseCode response_code = SetConfiguration(config);
        std::cout << "[Autopilot Manager] Initial provisioning finished with code " <<  response_code << std::endl;
    } else {
        std::cout << "[Autopilot Manager] Failed to init default config from " <<  _configPath << std::endl;
    }
}

AutopilotManager::ResponseCode AutopilotManager::SetConfiguration(AutopilotManagerConfig& config) {
    _autopilot_manager_enabled = config.autopilot_manager_enabled;
    _decision_maker_input_type = config.decision_maker_input_type;
    _simple_collision_avoid_distance_threshold = config.simple_collision_avoid_distance_threshold;
    _simple_collision_avoid_distance_on_condition_true = config.simple_collision_avoid_distance_on_condition_true;
    _simple_collision_avoid_distance_on_condition_false = config.simple_collision_avoid_distance_on_condition_false;

    return ResponseCode::SUCCEED;
}

AutopilotManager::ResponseCode AutopilotManager::GetConfiguration(AutopilotManagerConfig& config) {
    config.autopilot_manager_enabled = _autopilot_manager_enabled;
    config.decision_maker_input_type = _decision_maker_input_type;
    config.simple_collision_avoid_distance_threshold = _simple_collision_avoid_distance_threshold;
    config.simple_collision_avoid_distance_on_condition_true = _simple_collision_avoid_distance_on_condition_true;
    config.simple_collision_avoid_distance_on_condition_false = _simple_collision_avoid_distance_on_condition_false;

    return ResponseCode::SUCCEED;
}
