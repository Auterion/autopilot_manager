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

#include <AutopilotManager.hpp>

namespace {
const auto METHOD_GET_CONFIG = "get_config";
const auto METHOD_SET_CONFIG = "set_config";
}  // namespace

AutopilotManager::AutopilotManager(const std::string& mavlinkPort, const std::string& configPath = "",
                                   const std::string& customActionConfigPath = "")
    : _obstacle_avoidance_enabled(false),
      _mavlink_port(mavlinkPort),
      _config_path(configPath.empty() ? _config_path : configPath),
      _custom_action_config_path(customActionConfigPath.empty() ? _custom_action_config_path : customActionConfigPath) {
    initialProvisioning();

    start();
}

AutopilotManager::~AutopilotManager() {
    _interrupt_received.store(true, std::memory_order_relaxed);

    _sensor_manager_th.join();
    _collision_avoidance_manager_th.join();
    _landing_manager_th.join();
    _mission_manager_th.join();
    _mission_manager.reset();
    _sensor_manager.reset();
    _collision_avoidance_manager.reset();
    _landing_manager.reset();
}

auto AutopilotManager::HandleRequest(DBusMessage* request) -> DBusMessage* {
    DBusMessage* reply = nullptr;
    if (dbus_message_is_method_call(request, DBusInterface::INTERFACE_NAME, METHOD_GET_CONFIG)) {
        std::cout << "[Autopilot Manager] Received message: " << METHOD_GET_CONFIG << std::endl;
        if (!(reply = dbus_message_new_method_return(request))) {
            std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return" << std::endl;
        } else {
            AutopilotManagerConfig config;
            config.InitFromFile(_config_path);
            ResponseCode response_code = GetConfiguration(config);
            config.AppendToMessage(reply);
            dbus_message_append_args(reply, DBUS_TYPE_UINT32, &response_code, DBUS_TYPE_INVALID);
        }
    } else if (dbus_message_is_method_call(request, DBusInterface::INTERFACE_NAME, METHOD_SET_CONFIG)) {
        std::cout << "[Autopilot Manager] Received message: " << METHOD_SET_CONFIG << std::endl;
        if (!(reply = dbus_message_new_method_return(request))) {
            std::cerr << "[Autopilot Manager DBus Interface] Error: dbus_message_new_method_return" << std::endl;
        } else {
            AutopilotManagerConfig config;
            ResponseCode response_code;
            if (config.InitFromMessage(request)) {
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
        config.Print();
        ResponseCode response_code = SetConfiguration(config);
        std::cout << "[Autopilot Manager] Initial provisioning finished with code " << response_code << std::endl;
    } else {
        std::cout << "[Autopilot Manager] Failed to init default config from " << _config_path << std::endl;
    }
}

auto AutopilotManager::SetConfiguration(AutopilotManagerConfig config) -> AutopilotManager::ResponseCode {
    std::lock_guard<std::mutex> lock(_config_mutex);

    // General configurations
    _autopilot_manager_enabled = config.autopilot_manager_enabled;
    _decision_maker_input_type = config.decision_maker_input_type;

    // General configurations dependent on selected actions
    _script_to_call = config.script_to_call;
    _api_call = config.api_call;
    _local_position_offset_x = config.local_position_offset_x;
    _local_position_offset_y = config.local_position_offset_y;
    _local_position_offset_z = config.local_position_offset_z;
    _local_position_waypoint_x = config.local_position_waypoint_x;
    _local_position_waypoint_y = config.local_position_waypoint_y;
    _local_position_waypoint_z = config.local_position_waypoint_z;
    _global_position_offset_lat = config.global_position_offset_lat;
    _global_position_offset_lon = config.global_position_offset_lon;
    _global_position_offset_alt_amsl = config.global_position_offset_alt_amsl;
    _global_position_waypoint_lat = config.global_position_waypoint_lat;
    _global_position_waypoint_lon = config.global_position_waypoint_lon;
    _global_position_waypoint_alt_amsl = config.global_position_waypoint_alt_amsl;

    // Depth camera confguration
    _camera_offset_x = config.camera_offset_x;
    _camera_offset_y = config.camera_offset_y;
    _camera_yaw = config.camera_yaw;

    // Safe landing configurations
    _safe_landing_enabled = config.safe_landing_enabled;
    _safe_landing_area_square_size = config.safe_landing_area_square_size;
    _safe_landing_distance_to_ground = config.safe_landing_distance_to_ground;
    _safe_landing_on_no_safe_land = config.safe_landing_on_no_safe_land;
    _safe_landing_try_landing_after_action = config.safe_landing_try_landing_after_action;

    // Landing site search configurations
    _landing_site_search_speed = config.landing_site_search_speed;
    _landing_site_search_max_distance = config.landing_site_search_max_distance;
    _landing_site_search_min_height = config.landing_site_search_min_height;
    _landing_site_search_min_distance_after_abort = config.landing_site_search_min_distance_after_abort;
    _landing_site_search_arrival_radius = config.landing_site_search_arrival_radius;
    _landing_site_search_assess_time = config.landing_site_search_assess_time;
    _landing_site_search_strategy = config.landing_site_search_strategy;
    // Spiral search strategy
    _landing_site_search_spiral_spacing = config.landing_site_search_spiral_spacing;
    _landing_site_search_spiral_points = config.landing_site_search_spiral_points;

    // Simple collision avoidance configurations
    _simple_collision_avoid_enabled = config.simple_collision_avoid_enabled;
    _simple_collision_avoid_distance_threshold = config.simple_collision_avoid_distance_threshold;
    _simple_collision_avoid_action_on_condition_true = config.simple_collision_avoid_action_on_condition_true;

    if (_decision_maker_input_type == "SAFE_LANDING") {
        if (!static_cast<bool>(_safe_landing_enabled)) {
            return ResponseCode::SUCCEED_WITH_SAFE_LANDING_OFF;
        } else {
            return ResponseCode::SUCCEED_WITH_SAFE_LANDING_ON;
        }
    } else if (_decision_maker_input_type == "SIMPLE_COLLISION_AVOIDANCE") {
        if (!static_cast<bool>(_simple_collision_avoid_enabled)) {
            return ResponseCode::SUCCEED_WITH_COLL_AVOID_OFF;
        } else {
            return ResponseCode::SUCCEED_WITH_COLL_AVOID_ON;
        }
    }

    return ResponseCode::UNKNOWN;
}

auto AutopilotManager::GetConfiguration(AutopilotManagerConfig config) -> AutopilotManager::ResponseCode {
    std::lock_guard<std::mutex> lock(_config_mutex);

    // General configurations
    config.autopilot_manager_enabled = _autopilot_manager_enabled;
    config.decision_maker_input_type = _decision_maker_input_type;

    // General configurations dependent on selected actions
    config.script_to_call = _script_to_call;
    config.api_call = _api_call;
    config.local_position_offset_x = _local_position_offset_x;
    config.local_position_offset_y = _local_position_offset_y;
    config.local_position_offset_z = _local_position_offset_z;
    config.local_position_waypoint_x = _local_position_waypoint_x;
    config.local_position_waypoint_y = _local_position_waypoint_y;
    config.local_position_waypoint_z = _local_position_waypoint_z;
    config.global_position_offset_lat = _global_position_offset_lat;
    config.global_position_offset_lon = _global_position_offset_lon;
    config.global_position_offset_alt_amsl = _global_position_offset_alt_amsl;
    config.global_position_waypoint_lat = _global_position_waypoint_lat;
    config.global_position_waypoint_lon = _global_position_waypoint_lon;
    config.global_position_waypoint_alt_amsl = _global_position_waypoint_alt_amsl;

    // Depth camera confguration
    config.camera_offset_x = _camera_offset_x;
    config.camera_offset_y = _camera_offset_y;
    config.camera_yaw = _camera_yaw;

    // Safe landing configurations
    config.safe_landing_enabled = _safe_landing_enabled;
    config.safe_landing_area_square_size = _safe_landing_area_square_size;
    config.safe_landing_distance_to_ground = _safe_landing_distance_to_ground;
    config.safe_landing_on_no_safe_land = _safe_landing_on_no_safe_land;
    config.safe_landing_try_landing_after_action = _safe_landing_try_landing_after_action;

    // Landing site search configurations
    config.landing_site_search_speed = _landing_site_search_speed;
    config.landing_site_search_max_distance = _landing_site_search_max_distance;
    config.landing_site_search_min_height = _landing_site_search_min_height;
    config.landing_site_search_min_distance_after_abort = _landing_site_search_min_distance_after_abort;
    config.landing_site_search_arrival_radius = _landing_site_search_arrival_radius;
    config.landing_site_search_assess_time = _landing_site_search_assess_time;
    config.landing_site_search_strategy = _landing_site_search_strategy;
    // Spiral search strategy
    config.landing_site_search_spiral_spacing = _landing_site_search_spiral_spacing;
    config.landing_site_search_spiral_points = _landing_site_search_spiral_points;

    // Simple collision avoidance configurations
    config.simple_collision_avoid_enabled = _simple_collision_avoid_enabled;
    config.simple_collision_avoid_distance_threshold = _simple_collision_avoid_distance_threshold;
    config.simple_collision_avoid_action_on_condition_true = _simple_collision_avoid_action_on_condition_true;

    if (config.decision_maker_input_type == "SAFE_LANDING") {
        if (!config.safe_landing_enabled) {
            return ResponseCode::SUCCEED_WITH_SAFE_LANDING_OFF;
        } else {
            return ResponseCode::SUCCEED_WITH_SAFE_LANDING_ON;
        }
    } else if (config.decision_maker_input_type == "SIMPLE_COLLISION_AVOIDANCE") {
        if (!config.simple_collision_avoid_enabled) {
            return ResponseCode::SUCCEED_WITH_COLL_AVOID_OFF;
        } else {
            return ResponseCode::SUCCEED_WITH_COLL_AVOID_ON;
        }
    }

    return ResponseCode::UNKNOWN;
}

void AutopilotManager::start() {
    // Configure MAVSDK Mission Manager instance
    // Change configuration so the instance is treated as a mission computer
    mavsdk::Mavsdk::Configuration configuration(kDefaultSystemId, kMavCompIDOnBoardComputer3, true);
    configuration.set_component_description("Autopilot Manager");
    _mavsdk_mission_computer.set_configuration(configuration);

    mavsdk::ConnectionResult ret_comp = _mavsdk_mission_computer.add_any_connection("udp://:" + _mavlink_port);
    if (ret_comp == mavsdk::ConnectionResult::Success) {
        std::cout << "[Autopilot Manager] Waiting to discover vehicle from the mission computer side..." << std::endl;
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
        auto fut = prom.get_future();

        // We wait for new systems to be discovered, and use the one that has an autopilot
        _mavsdk_mission_computer.subscribe_on_new_system([&prom, this]() {
            const auto systems = _mavsdk_mission_computer.systems();
            auto system_it =
                std::find_if(systems.cbegin(), systems.cend(), [&](const auto& sys) { return sys->has_autopilot(); });
            if (system_it != systems.cend()) {
                std::cout << "[Autopilot Manager] Discovered autopilot!\n";
                prom.set_value(*system_it);
                // Unsubscribe again as we only want to find one system.
                _mavsdk_mission_computer.subscribe_on_new_system(nullptr);
            }
        });

        // Usually receives heartbeats at 1Hz, therefore it should find a
        // system after around 3 seconds. 10 secs is defined as a max to wait
        if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
            std::cerr << "[Autopilot Manager] No autopilot found, exiting...\n";
            exit(1);
        }

        // Get discovered system now
        const auto system = fut.get();

        // Start modules
        start_sensor_manager(system);
        start_collision_avoidance_manager();
        start_landing_manager(system);
        start_mission_manager(system);

        // MAVLink passthrough
        _mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(system);

        // Create reusable heartbeat message
        create_avoidance_mavlink_heartbeat_message();

        // Run the main Autopilot Manager main loop
        run();

    } else {
        std::cerr << "[Autopilot Manager] Failed to connect to port! Exiting..." << _mavlink_port << std::endl;
        exit(1);
    }
}

void AutopilotManager::start_sensor_manager(std::shared_ptr<mavsdk::System> mavsdk_system) {
    _sensor_manager = std::make_shared<SensorManager>(mavsdk_system);
    _sensor_manager->init();
    _sensor_manager->set_camera_static_tf(_camera_offset_x, _camera_offset_y, _camera_yaw);
    _sensor_manager_th = std::thread(&AutopilotManager::run_sensor_manager, this);
}

void AutopilotManager::start_collision_avoidance_manager() {
    _collision_avoidance_manager = std::make_shared<CollisionAvoidanceManager>();

    _collision_avoidance_manager->setConfigUpdateCallback([this]() {
        std::lock_guard<std::mutex> lock(_config_mutex);

        CollisionAvoidanceManager::CollisionAvoidanceManagerConfiguration config;
        config.autopilot_manager_enabled = _autopilot_manager_enabled;
        config.simple_collision_avoid_enabled = _simple_collision_avoid_enabled;
        return config;
    });

    _collision_avoidance_manager->getDownsampledDepthDataCallback([this]() {
        std::lock_guard<std::mutex> lock(_downsampled_depth_callback_mutex);
        return _sensor_manager->get_lastest_downsampled_depth();
    });

    _collision_avoidance_manager->init();
    _collision_avoidance_manager_th = std::thread(&AutopilotManager::run_collision_avoidance_manager, this);
}

void AutopilotManager::start_landing_manager(std::shared_ptr<mavsdk::System> mavsdk_system) {
    _landing_manager = std::make_shared<LandingManager>(mavsdk_system);

    _landing_manager->setConfigUpdateCallback([this]() {
        std::lock_guard<std::mutex> lock(_config_mutex);

        LandingManager::LandingManagerConfiguration config;
        config.autopilot_manager_enabled = _autopilot_manager_enabled;
        config.safe_landing_enabled = _safe_landing_enabled;
        config.safe_landing_area_square_size = _safe_landing_area_square_size;
        config.safe_landing_distance_to_ground = _safe_landing_distance_to_ground;
        return config;
    });

    _landing_manager->getDownsampledDepthDataCallback([this]() {
        std::lock_guard<std::mutex> lock(_downsampled_depth_callback_mutex);
        return _sensor_manager->get_lastest_downsampled_depth();
    });

    _landing_manager->init();
    _landing_manager_th = std::thread(&AutopilotManager::run_landing_manager, this);
}

void AutopilotManager::start_mission_manager(std::shared_ptr<mavsdk::System> mavsdk_system) {
    _mission_manager = std::make_shared<MissionManager>(mavsdk_system, _custom_action_config_path);

    // Init the callback for setting the Mission Manager parameters
    _mission_manager->setConfigUpdateCallback([this]() {
        std::lock_guard<std::mutex> lock(_config_mutex);

        MissionManager::MissionManagerConfiguration config;
        config.autopilot_manager_enabled = _autopilot_manager_enabled;
        config.decision_maker_input_type = _decision_maker_input_type;
        config.script_to_call = _script_to_call;
        config.api_call = _api_call;
        config.local_position_offset_x = _local_position_offset_x;
        config.local_position_offset_y = _local_position_offset_y;
        config.local_position_offset_z = _local_position_offset_z;
        config.local_position_waypoint_x = _local_position_waypoint_x;
        config.local_position_waypoint_y = _local_position_waypoint_y;
        config.local_position_waypoint_z = _local_position_waypoint_z;
        config.global_position_offset_lat = _global_position_offset_lat;
        config.global_position_offset_lon = _global_position_offset_lon;
        config.global_position_offset_alt_amsl = _global_position_offset_alt_amsl;
        config.global_position_waypoint_lat = _global_position_waypoint_lat;
        config.global_position_waypoint_lon = _global_position_waypoint_lon;
        config.global_position_waypoint_alt_amsl = _global_position_waypoint_alt_amsl;
        config.safe_landing_enabled = _safe_landing_enabled;
        config.safe_landing_distance_to_ground = _safe_landing_distance_to_ground;
        config.safe_landing_on_no_safe_land = _safe_landing_on_no_safe_land;
        config.safe_landing_try_landing_after_action = _safe_landing_try_landing_after_action;
        config.landing_site_search_speed = _landing_site_search_speed;
        config.landing_site_search_max_distance = _landing_site_search_max_distance;
        config.landing_site_search_min_height = _landing_site_search_min_height;
        config.landing_site_search_min_distance_after_abort = _landing_site_search_min_distance_after_abort;
        config.landing_site_search_arrival_radius = _landing_site_search_arrival_radius;
        config.landing_site_search_assess_time = _landing_site_search_assess_time;
        config.landing_site_search_strategy = _landing_site_search_strategy;
        config.landing_site_search_spiral_spacing = _landing_site_search_spiral_spacing;
        config.landing_site_search_spiral_points = _landing_site_search_spiral_points;
        config.simple_collision_avoid_enabled = _simple_collision_avoid_enabled;
        config.simple_collision_avoid_distance_threshold = _simple_collision_avoid_distance_threshold;
        config.simple_collision_avoid_action_on_condition_true = _simple_collision_avoid_action_on_condition_true;
        return config;
    });

    // Init the callback for getting the latest distance to obstacle
    _mission_manager->getDistanceToObstacleCallback([this]() {
        std::lock_guard<std::mutex> lock(_distance_to_obstacle_mutex);
        return _collision_avoidance_manager->get_latest_distance();
    });

    // Init the callback for getting the latest landing condition state
    _mission_manager->getCanLandStateCallback([this]() {
        std::lock_guard<std::mutex> lock(_landing_condition_state_mutex);
        return _landing_manager->get_latest_landing_condition_state();
    });

    // Init the callback for getting the landing condition state at a particular position
    _mission_manager->getCanLandAtPositionStateCallback([this](float x, float y) {
        std::lock_guard<std::mutex> lock(_landing_condition_state_mutex);
        return _landing_manager->get_landing_condition_state_at_position(x, y);
    });

    // Init the callback for getting the latest landing condition state
    _mission_manager->getHeightAboveObstacleCallback([this]() {
        std::lock_guard<std::mutex> lock(_height_above_obstacle_mutex);
        return _landing_manager->get_latest_height_above_obstacle();
    });

    // Init and run the Mission Manager
    _mission_manager->init();
    _mission_manager_th = std::thread(&AutopilotManager::run_mission_manager, this);
}

void AutopilotManager::run_sensor_manager() {
    // Run the Sensor Manager node
    _sensor_manager->run();
}

void AutopilotManager::run_collision_avoidance_manager() {
    // Run the Collision Avoidance Manager node
    _collision_avoidance_manager->run();
}

void AutopilotManager::run_landing_manager() {
    // Run the Landing Manager node
    _landing_manager->run();
}

void AutopilotManager::run_mission_manager() {
    // Run the Mission Manager node
    _mission_manager->run();
}

void AutopilotManager::run() {
    while (!_interrupt_received) {
        // Check if obstacle avoidance is enabled
        update_obstacle_avoidance_enabled();

        if (_safe_landing_enabled) {
            // Send avoidance heartbeat
            const bool sm_healthy = _sensor_manager->isHealthy();
            const bool lm_healthy = _landing_manager->isHealthy();
            const bool mm_healthy = _mission_manager->isHealthy();

            if (sm_healthy && lm_healthy && mm_healthy) {
                _mavlink_passthrough->send_message(_avoidance_heartbeat_message);
            }
        }

        // Update at 1Hz
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void AutopilotManager::update_obstacle_avoidance_enabled() {
    const bool should_be_enabled = _mission_manager->is_obstacle_avoidance_enabled();
    const bool should_change = should_be_enabled != _obstacle_avoidance_enabled;

    if (!should_change) {
        return;
    }

    _obstacle_avoidance_enabled = should_be_enabled;
    std::cout << autopilotManagerOut << "Obstacle avoidance has been " << (should_be_enabled ? "enabled" : "disabled")
              << " in PX4." << std::endl;

    // Update the modules that use the OA-enabled parameter
    _landing_manager->set_obstacle_avoidance_enabled(_obstacle_avoidance_enabled);
    _sensor_manager->set_obstacle_avoidance_enabled(_obstacle_avoidance_enabled);
}

void AutopilotManager::create_avoidance_mavlink_heartbeat_message() {
    mavlink_heartbeat_t heartbeat;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    mavlink_msg_heartbeat_encode(1, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &_avoidance_heartbeat_message, &heartbeat);
}
