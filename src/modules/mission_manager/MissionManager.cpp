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
 * @brief Mission Manager
 * @file MissionManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <MissionManager.hpp>
#include <atomic>
#include <future>
#include <iostream>
#include <string>
#include <utility>

static std::atomic<bool> int_signal{false};

MissionManager::MissionManager(std::shared_ptr<mavsdk::System> mavsdk_system,
                               const std::string& path_to_custom_action_file)
    : _config_update_callback([]() { return MissionManagerConfiguration{}; }),
      _path_to_custom_action_file{std::move(path_to_custom_action_file)},
      _mission_manager_config{},
      _mavsdk_system{std::move(mavsdk_system)},
      _action_triggered{false},
      _in_air{false},
      _landing{false},
      _on_ground{false},
      _current_latitude{0.0},
      _current_longitude{0.0},
      _current_altitude_amsl{0.0},
      _new_latitude{0.0},
      _new_longitude{0.0},
      _new_altitude_amsl{0.0},
      _previously_set_waypoint_latitude{0.0},
      _previously_set_waypoint_longitude{0.0},
      _previously_set_waypoint_altitude_amsl{0.0} {}

MissionManager::~MissionManager() { deinit(); }

void MissionManager::init() {
    std::cout << missionManagerOut << "Started!" << std::endl;

    // Actions are processed and executed in the Mission Manager decion maker
    _action = std::make_shared<mavsdk::Action>(_mavsdk_system);

    // Telemetry data checks are fundamental for proper execution
    _telemetry = std::make_shared<mavsdk::Telemetry>(_mavsdk_system);

    // Bring up a ServerUtility instance to allow sending status messages
    _server_utility = std::make_shared<mavsdk::ServerUtility>(_mavsdk_system);

    _custom_action_handler =
        std::make_shared<CustomActionHandler>(_mavsdk_system, _telemetry, _path_to_custom_action_file);
}

void MissionManager::deinit() {
    int_signal.store(true, std::memory_order_relaxed);

    _decision_maker_th.join();
    _global_origin_reference_th.join();
    _custom_action_handler.reset();
}

void MissionManager::run() {
    // Start the desicion maker thread
    _decision_maker_th = std::thread(&MissionManager::decision_maker_run, this);

    // Start custom action handler
    if (_custom_action_handler->start()) {
        _custom_action_handler->run();
    }
}

void MissionManager::set_global_position_reference() {
    bool result{false};
    std::string status{};

    while (!_get_gps_origin_success && !int_signal && !(_is_global_position_ok && _is_home_position_ok)) {
        // Get global origin to set the reference global position
        auto cmd_result = _telemetry->get_gps_global_origin();

        if (cmd_result.first == mavsdk::Telemetry::Result::Success) {
            std::cout << std::string(missionManagerOut) + "Successfully received a GPS_GLOBAL_ORIGIN MAVLink message"
                      << std::endl;
            _get_gps_origin_success = true;

            if (cmd_result.second.latitude_deg != 0.0 && cmd_result.second.longitude_deg != 0.0) {
                _ref_latitude = cmd_result.second.latitude_deg;
                _ref_longitude = cmd_result.second.longitude_deg;
                _ref_altitude = cmd_result.second.altitude_m;

                status = std::string(missionManagerOut) +
                         "Global position reference initialiazed: Latitude: " + std::to_string(_ref_latitude) +
                         " deg | Longitude: " + std::to_string(_ref_longitude) +
                         " deg | Altitude (AMSL): " + std::to_string(_ref_altitude) + " meters";
                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);

                result = true;

            } else {
                status = std::string(missionManagerOut) +
                         "Failed to set the global origin reference because the received values are invalid.";
                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Error, status);
            }

        } else if (cmd_result.first == mavsdk::Telemetry::Result::Timeout) {
            std::cout << std::string(missionManagerOut) +
                             "GPS_GLOBAL_ORIGIN stream request timeout. Message not yet received. Retrying..."
                      << std::endl;
        } else {
            std::cout << std::string(missionManagerOut) + "GPS_GLOBAL_ORIGIN stream request failed. Retrying..."
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << status << std::endl;
    _global_origin_reference_set = result;
}

mavsdk::geometry::CoordinateTransformation::GlobalCoordinate MissionManager::get_global_position_from_local_offset(
    const double& offset_x, const double& offset_y) const {
    const double local_position_x =
        (std::cos(_current_yaw) * offset_x - std::sin(_current_yaw) * offset_y) + _current_pos_x;
    const double local_position_y =
        (std::sin(_current_yaw) * offset_x + std::cos(_current_yaw) * offset_y) + _current_pos_y;

    const mavsdk::geometry::CoordinateTransformation ct({_ref_latitude, _ref_longitude});
    return ct.global_from_local({local_position_x, local_position_y});
}

void MissionManager::set_new_waypoint(const double& lat, const double& lon, const double& alt_amsl) {
    _new_latitude = lat;
    _new_longitude = lon;
    _new_altitude_amsl = alt_amsl;
}

bool MissionManager::arrived_to_new_waypoint() {
    if ((std::abs(_new_latitude - _current_latitude) <= 1.0E-5) &&
        (std::abs(_new_longitude - _current_longitude) <= 1.0E-5) &&
        (std::abs(_new_altitude_amsl - _current_altitude_amsl) <= 1.0)) {  // ~1 meter acceptance radius
        return true;
    }

    return false;
}

void MissionManager::handle_safe_landing(std::chrono::time_point<std::chrono::system_clock> now) {
    std::unique_lock<std::mutex> lock(mission_manager_config_mtx);
    const bool safe_landing_enabled = _mission_manager_config.safe_landing_enabled;
    const float safe_landing_distance_to_ground = _mission_manager_config.safe_landing_distance_to_ground;
    const bool safe_landing_try_landing_after_action = _mission_manager_config.safe_landing_try_landing_after_action;
    const std::string safe_landing_on_no_safe_land = _mission_manager_config.safe_landing_on_no_safe_land;
    const double global_position_waypoint_lat = _mission_manager_config.global_position_waypoint_lat;
    const double global_position_waypoint_lon = _mission_manager_config.global_position_waypoint_lon;
    const double global_position_waypoint_alt_amsl = _mission_manager_config.global_position_waypoint_alt_amsl;
    const double local_position_offset_x = _mission_manager_config.local_position_offset_x;
    const double local_position_offset_y = _mission_manager_config.local_position_offset_y;
    const double local_position_offset_z = _mission_manager_config.local_position_offset_z;
    lock.unlock();

    const uint8_t safe_landing_state = _landing_condition_state_update_callback();
    const float height_above_obstacle = _height_above_obstacle_update_callback();

    if (safe_landing_enabled) {
        if (_landing && !_on_ground) {
            std::string status{};

            // if (height_above_obstacle <= (safe_landing_distance_to_ground - 0.2) && !_action_triggered) {
            if (!_action_triggered) {
                if (safe_landing_state == 0 /*eLandingMapperState::UNHEALTHY*/) {
                    // If the safe landing status is unhealthy, then hold position.
                    _action->hold();

                    status = std::string(missionManagerOut) + "Safe landing system not healthy. Holding position...";
                    _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                    std::cout << status << std::endl;

                    _action_triggered = true;
                    _last_time = now;

                } else if (height_above_obstacle > 0.5 && safe_landing_state == 1 /*eLandingMapperState::UNKNOWN*/) {
                    // If the vehicle is bellow the defined maximum distance to ground to determine if it is
                    // safe to land or not, then it will still try to land until it reaches an height that
                    // allows it to determine if it can land or not. Otherwise, it holds position.
                    _action->hold();

                    status =
                        std::string(missionManagerOut) + "Can't determine if it is safe to land. Holding position...";
                    _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                    std::cout << status << std::endl;

                    _action_triggered = true;
                    _last_time = now;

                } else if (safe_landing_state == 3 /*eLandingMapperState::CAN_NOT_LAND*/) {
                    std::cout << "Cannot land! -----------------------" << std::endl;
                    if (safe_landing_on_no_safe_land == "HOLD") {
                        _action->hold();

                        status = std::string(missionManagerOut) + "Position hold triggered for Safe Landing";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "RTL") {
                        _action->return_to_launch();

                        status = std::string(missionManagerOut) + "RTL triggered for Safe Landing";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "MOVE_XYZ_WRT_CURRENT") {
                        // get the global origin from the FMU and set the reference
                        if (_global_origin_reference_set) {
                            const auto waypoint =
                                get_global_position_from_local_offset(local_position_offset_x, local_position_offset_y);
                            const double waypoint_altitude = _current_altitude_amsl + local_position_offset_z;

                            _action->goto_location(waypoint.latitude_deg, waypoint.longitude_deg, waypoint_altitude,
                                                   NAN);

                            set_new_waypoint(waypoint.latitude_deg, waypoint.longitude_deg, waypoint_altitude);

                            status =
                                std::string(missionManagerOut) +
                                "Moving XYZ WRT to current vehicle position triggered for Safe Landing. Heading to "
                                "determined "
                                "Latitude " +
                                std::to_string(waypoint.latitude_deg) + " deg, Longitude " +
                                std::to_string(waypoint.longitude_deg) + " deg, Altitude (AMSL) " +
                                std::to_string(waypoint_altitude) + " meters";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);

                        } else {
                            _action->hold();

                            status = std::string(missionManagerOut) + "Holding position...";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        }

                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "GO_TO_WAYPOINT_XYZ") {
                        _action->hold();

                        status =
                            std::string(missionManagerOut) +
                            "GO_TO_WAYPOINT_XYZ action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "MOVE_LLA_WRT_CURRENT") {
                        _action->hold();

                        status =
                            std::string(missionManagerOut) +
                            "MOVE_LLA_WRT_CURRENT action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "GO_TO_WAYPOINT") {
                        std::string status{};

                        // If this action run previously, but the user didn't change the waypoint online after the
                        // action was triggered, the previous waypoint will match the current waypoint. If that's the
                        // case enfore an RTL so to avoid the vehicle getting stuck trying to land in a place it can't
                        // land
                        if ((std::abs(global_position_waypoint_lat - _current_latitude) <= 1.0E-5) &&
                            (std::abs(global_position_waypoint_lon - _current_longitude) <= 1.0E-5) &&
                            (std::abs(_previously_set_waypoint_altitude_amsl - _current_altitude_amsl) <= 1.0)) {
                            _action->return_to_launch();

                            status =
                                std::string(missionManagerOut) +
                                "Go-To Global Position Waypoint not triggered for Safe Landing, as the waypoint set "
                                "is the same as the "
                                "global position of the vehicle."
                                "RTL triggered instead...";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);

                        } else {
                            // get the global origin from the FMU and set the reference
                            if (_global_origin_reference_set) {
                                // then send the DO_REPOSITION
                                _action->goto_location(global_position_waypoint_lat, global_position_waypoint_lon,
                                                       global_position_waypoint_alt_amsl, NAN);

                                set_new_waypoint(global_position_waypoint_lat, global_position_waypoint_lon,
                                                 global_position_waypoint_alt_amsl);

                                status =
                                    std::string(missionManagerOut) +
                                    "Go-To Global Position Waypoint triggered for Safe Landing. Heading to Latitude " +
                                    std::to_string(global_position_waypoint_lat) + " deg, Longitude " +
                                    std::to_string(global_position_waypoint_lon) + " deg, Altitude (AMSL) " +
                                    std::to_string(global_position_waypoint_alt_amsl) + " meters";
                                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);

                            } else {
                                _action->hold();

                                status = std::string(missionManagerOut) + "Holding position...";
                                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning,
                                                                  status);
                            }
                        }

                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "SCRIPT_CALL") {
                        _action->hold();

                        status = std::string(missionManagerOut) +
                                 "SCRIPT_CALL action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "API_CALL") {
                        _action->hold();

                        status = std::string(missionManagerOut) +
                                 "API_CALL action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;
                    }

                    _action_triggered = true;
                    _last_time = now;
                }
            }
        }

        // Check if new waypoint was set as part of the action
        if (std::isfinite(_new_latitude) && std::isfinite(_new_longitude) && std::isfinite(_new_altitude_amsl)) {
            // check if we arrived to the newly set waypoint
            if (arrived_to_new_waypoint()) {
                // if the user set it wants to try landing again after arriving to the waypoint
                // then land and unset the new waypoint
                if (safe_landing_try_landing_after_action) {
                    _action->land();
                    set_new_waypoint(NAN, NAN, NAN);
                }
            }
        }
    }
}

void MissionManager::handle_simple_collision_avoidance(std::chrono::time_point<std::chrono::system_clock> now) {
    if (_mission_manager_config.simple_collision_avoid_enabled != 0U) {
        // std::cout << "Depth measured: " << _distance_to_obstacle_update_callback()
        //           << " | threshold: " << _mission_manager_config.simple_collision_avoid_distance_threshold
        //           << " | in air: " << _in_air << " | is action triggered? " << std::boolalpha
        //           << _action_triggered << std::endl;

        if (std::isfinite(_distance_to_obstacle_update_callback()) &&
            _distance_to_obstacle_update_callback() <=
                _mission_manager_config.simple_collision_avoid_distance_threshold &&
            _in_air && !_action_triggered) {  // only trigger the condition when the vehicle is in-air
            if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "HOLD") {
                _action->hold();
                std::cout << std::string(missionManagerOut) << "Position hold triggered for Simple Obstacle Avoidance"
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "RTL") {
                _action->return_to_launch();
                std::cout << std::string(missionManagerOut) << "RTL triggered for Simple Obstacle Avoidance"
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "MOVE_XYZ_WRT_CURRENT") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "MOVE_XYZ_WRT_CURRENT action currently not supported for Simple Obstacle Avoidance. "
                             "Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "GO_TO_WAYPOINT_XYZ") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "GO_TO_WAYPOINT_XYZ action currently not supported for Simple Obstacle Avoidance. Holding "
                             "position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "MOVE_LLA_WRT_CURRENT") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "MOVE_LLA_WRT_CURRENT action currently not supported for Simple Obstacle Avoidance. "
                             "Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "GO_TO_WAYPOINT") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "GO_TO_WAYPOINT action currently not supported for Simple Obstacle Avoidance. Holding "
                             "position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "LAND") {
                _action->land();
                std::cout << std::string(missionManagerOut) << "Land triggered for Simple Obstacle Avoidance"
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "SCRIPT_CALL") {
                _action->hold();
                std::cout
                    << std::string(missionManagerOut)
                    << "SCRIPT_CALL action currently not supported for Simple Obstacle Avoidance. Holding position..."
                    << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "API_CALL") {
                _action->hold();
                std::cout
                    << std::string(missionManagerOut)
                    << "API_CALL action currently not supported for Simple Obstacle Avoidance. Holding position..."
                    << std::endl;
            }

            _action_triggered = true;
            _last_time = now;
        }
    }
}

void MissionManager::decision_maker_run() {
    // Init action trigger timer
    _last_time = std::chrono::system_clock::now();

    // Get global position
    _telemetry->subscribe_position([this](mavsdk::Telemetry::Position position) {
        _current_latitude = position.latitude_deg;
        _current_longitude = position.longitude_deg;
        _current_altitude_amsl = position.absolute_altitude_m;
    });

    // Get global and home positions health
    _telemetry->subscribe_health([this](mavsdk::Telemetry::Health health) {
        _is_global_position_ok = health.is_global_position_ok;
        _is_home_position_ok = health.is_home_position_ok;
    });

    // Get local position
    _telemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed position_velocity) {
        _current_pos_x = position_velocity.position.north_m;
        _current_pos_y = position_velocity.position.east_m;
        _current_pos_z = position_velocity.position.down_m;
    });

    // Get the GPS global origin and set the global origin reference
    _global_origin_reference_th = std::thread(&MissionManager::set_global_position_reference, this);

    // Get yaw
    _telemetry->subscribe_attitude_euler(
        [this](mavsdk::Telemetry::EulerAngle euler_angle) { _current_yaw = euler_angle.yaw_deg * M_PI / 180.0; });

    // Get the landing state so we know when the vehicle is in-air, landing or in-ground
    _telemetry->subscribe_landed_state([this](mavsdk::Telemetry::LandedState landed_state) {
        switch (landed_state) {
            case mavsdk::Telemetry::LandedState::InAir:
                _in_air = true;
                _landing = false;
                _on_ground = false;
                break;
            case mavsdk::Telemetry::LandedState::Landing:
                _in_air = false;
                _landing = true;
                _on_ground = false;
                break;
            case mavsdk::Telemetry::LandedState::Unknown:
            case mavsdk::Telemetry::LandedState::OnGround:
                _in_air = false;
                _landing = false;
                _on_ground = true;
                break;
            case mavsdk::Telemetry::LandedState::TakingOff:
            default:
                _in_air = false;
                _landing = false;
                _on_ground = false;
        }
    });

    while (!int_signal) {
        // Update configuration at each iteration
        _mission_manager_config = _config_update_callback();

        if (_mission_manager_config.autopilot_manager_enabled) {
            auto now = std::chrono::system_clock::now();

            if (_mission_manager_config.decision_maker_input_type == "SAFE_LANDING") {
                handle_safe_landing(now);
            } else if (_mission_manager_config.decision_maker_input_type == "SIMPLE_COLLISION_AVOIDANCE") {
                handle_simple_collision_avoidance(now);
            }

            // After an action is triggered, we give it 5 seconds to process it before retrying
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_time).count() >= 5000) {
                _action_triggered = false;
            }
        }

        // Decision maker runs at 20hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
