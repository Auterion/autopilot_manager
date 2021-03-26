/****************************************************************************
 *
 *   Copyright (c) 2021 Auterion AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Auterion nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Autopilot Manager
 * @file AutopilotManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/signalfd.h>
#include <unistd.h>

#include <AutopilotManager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

namespace {
const auto METHOD_GET_CONFIG = "get_config";
const auto METHOD_SET_CONFIG = "set_config";
}  // namespace

AutopilotManager::AutopilotManager(const std::string& mavlinkPort, const std::string& configPath = "",
                                   const std::string& customActionConfigPath = "")
    : _mavlink_port(mavlinkPort),
      _config_path(configPath.empty() ? _config_path : configPath),
      _custom_action_config_path(customActionConfigPath.empty() ? _custom_action_config_path : customActionConfigPath) {
    initialProvisioning();

    if (_autopilot_manager_enabled) {
        std::cout << "[Autopilot Manager] Status: Enabled!" << std::endl;

        start();
    }
}

AutopilotManager::~AutopilotManager() {
    _sensor_manager_th.join();
    _mission_manager.reset();
    _sensor_manager.reset();
}

DBusMessage* AutopilotManager::HandleRequest(DBusMessage* message) {
    DBusMessage* reply = nullptr;
    if (dbus_message_is_method_call(message, DBusInterface::INTERFACE_NAME, METHOD_GET_CONFIG)) {
        std::cout << "[Autopilot Manager] Received message: " << METHOD_GET_CONFIG << std::endl;
        if (!(reply = dbus_message_new_method_return(message))) {
            std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return" << std::endl;
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
            std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return" << std::endl;
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
        std::cout << "[Autopilot Manager] Initial provisioning finished with code " << response_code << std::endl;
    } else {
        std::cout << "[Autopilot Manager] Failed to init default config from " << _config_path << std::endl;
    }
}

AutopilotManager::ResponseCode AutopilotManager::SetConfiguration(AutopilotManagerConfig& config) {
    std::lock_guard<std::mutex> lock(_config_mutex);
    _autopilot_manager_enabled = config.autopilot_manager_enabled;
    _decision_maker_input_type = config.decision_maker_input_type;
    _simple_collision_avoid_enabled = config.simple_collision_avoid_enabled;
    _simple_collision_avoid_distance_threshold = config.simple_collision_avoid_distance_threshold;
    _simple_collision_avoid_distance_on_condition_true = config.simple_collision_avoid_distance_on_condition_true;
    _simple_collision_avoid_distance_on_condition_false = config.simple_collision_avoid_distance_on_condition_false;

    if (!_simple_collision_avoid_enabled) {
        return ResponseCode::SUCCEED_WITH_COLL_AVOID_OFF;
    } else {
        return ResponseCode::SUCCEED_WITH_COLL_AVOID_ON;
    }

    return ResponseCode::UNKNOWN;
}

AutopilotManager::ResponseCode AutopilotManager::GetConfiguration(AutopilotManagerConfig& config) {
    std::lock_guard<std::mutex> lock(_config_mutex);
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

void AutopilotManager::start() {
    // Configure MAVSDK Mission Manager instance
    // Change configuration so the instance is treated as a mission computer
    _mavsdk_mission_computer.set_configuration(
        mavsdk::Mavsdk::Configuration(mavsdk::Mavsdk::Configuration::UsageType::CompanionComputer));

    auto system = std::shared_ptr<mavsdk::System>{nullptr};

    mavsdk::ConnectionResult ret_comp = _mavsdk_mission_computer.add_any_connection("udp://:" + _mavlink_port);
    if (ret_comp == mavsdk::ConnectionResult::Success) {
        std::cout << "[Autopilot Manager] Waiting to discover vehicle from the mission computer side..." << std::endl;
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
        auto fut = prom.get_future();

        // We wait for new systems to be discovered, once we find one that has an
        // autopilot, we decide to use it
        _mavsdk_mission_computer.subscribe_on_new_system([&prom, this]() {
            for (const auto& sys : _mavsdk_mission_computer.systems()) {
                if (sys->has_autopilot()) {
                    std::cout << "[Autopilot Manager] Discovered autopilot!\n";
                    prom.set_value(sys);
                    // Unsubscribe again as we only want to find one system.
                    _mavsdk_mission_computer.subscribe_on_new_system(nullptr);
                    break;
                }
            }
        });

        // We usually receive heartbeats at 1Hz, therefore we should find a
        // system after around 10 seconds max
        if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
            std::cerr << "[Autopilot Manager] No autopilot found, exiting...\n";
            exit(1);
        }

        // Get discovered system now
        auto system = fut.get();

        // Create and run the Sensor Manager
        _sensor_manager = std::make_shared<SensorManager>();
        _sensor_manager_th = std::thread(&AutopilotManager::run_sensor_manager, this);

        // Create and init Mission Manager
        _mission_manager = std::make_shared<MissionManager>(system, _custom_action_config_path);

        // Init the callback for setting the Mission Manager parameters
        _mission_manager->setConfigUpdateCallback([this]() {
            std::lock_guard<std::mutex> lock(_config_mutex);
            return MissionManager::MissionManagerConfiguration{
                .decision_maker_input_type = _decision_maker_input_type,
                .simple_collision_avoid_enabled = _simple_collision_avoid_enabled,
                .simple_collision_avoid_distance_threshold = _simple_collision_avoid_distance_threshold,
                .simple_collision_avoid_distance_on_condition_true = _simple_collision_avoid_distance_on_condition_true,
                .simple_collision_avoid_distance_on_condition_false =
                    _simple_collision_avoid_distance_on_condition_false};
        });

        // Init the callback for getting the latest distance to obstacle
        _mission_manager->getDistanceToObstacleCallback([this]() {
            std::lock_guard<std::mutex> lock(_distance_to_obstacle_mutex);
            return _sensor_manager->get_latest_depth();
        });

        // Run the Mission Manager
        _mission_manager->init();
        _mission_manager->run();

    } else {
        std::cerr << "[Autopilot Manager] Failed to connect to port! Exiting..." << _mavlink_port << std::endl;
        exit(1);
    }
}

void AutopilotManager::run_sensor_manager() {
    // Init the Sensor Manager node
    spin(_sensor_manager);
}
