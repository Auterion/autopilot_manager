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

using namespace std::placeholders;
using namespace std::chrono_literals;

static std::atomic<bool> int_signal{false};

MissionManager::MissionManager(std::shared_ptr<mavsdk::System> mavsdk_system, mavsdk::Mavsdk* mavsdk,
                               const std::string& path_to_custom_action_file)
    : Node("mission_manager"),
      _config_update_callback([]() { return MissionManagerConfiguration{}; }),
      _path_to_custom_action_file{std::move(path_to_custom_action_file)},
      _mission_manager_config{},
      _mavsdk{mavsdk},
      _mavsdk_system{std::move(mavsdk_system)},
      _action_triggered{false},
      _global_origin_reference_set{false},
      _is_stationary_debounce_counter{0},
      _current_latitude{0.0},
      _current_longitude{0.0},
      _current_altitude_amsl{0.0},
      _new_latitude{NAN},
      _new_longitude{NAN},
      _new_altitude_amsl{NAN},
      _new_x{NAN},
      _new_y{NAN},
      _new_z{NAN},
      _new_yaw{NAN},
      _previously_set_waypoint_latitude{0.0},
      _previously_set_waypoint_longitude{0.0},
      _previously_set_waypoint_altitude_amsl{0.0},
      _original_max_speed{-1.0},
      _landing_planner{},
      _was_mission_paused{false},
      _was_landing_paused{false},
      _landing_latitude_deg{NAN},
      _landing_longitude_deg{NAN},
      _landing_altitude_m{NAN},
      _landing_waypoint_id{-1},
      _time_last_traj{this->now()},
      _got_traj{true},
      _frequency_traj("traj in") {}

MissionManager::~MissionManager() { deinit(); }

void MissionManager::init() {
    std::cout << missionManagerOut << "Started!" << std::endl;

    // Actions are processed and executed in the Mission Manager decion maker
    _action = std::make_shared<mavsdk::Action>(_mavsdk_system);
    _action->set_maximum_speed(5.f);

    _mission_raw = std::make_shared<mavsdk::MissionRaw>(_mavsdk_system);

    // Telemetry data checks are fundamental for proper execution
    _telemetry = std::make_shared<mavsdk::Telemetry>(_mavsdk_system);

    // Bring up a ServerUtility instance to allow sending status messages
    _server_utility = std::make_shared<mavsdk::ServerUtility>(_mavsdk_system);

    _mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(_mavsdk_system);

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

    // Get the GPS global origin and set the global origin reference
    _global_origin_reference_th = std::thread(&MissionManager::set_global_position_reference, this);

    // Start custom action handler
    if (_custom_action_handler->start()) {
        _custom_action_handler->run();
    }

    _mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS,
                                                  std::bind(&MissionManager::on_mavlink_trajectory_message, this, _1));

    rclcpp::spin(shared_from_this());
}

void MissionManager::on_mavlink_trajectory_message(const mavlink_message_t& _message) {
    if (_message.compid == MAV_COMP_ID_OBSTACLE_AVOIDANCE) {
        return;
    }

    _time_last_traj = this->now();

    const bool is_pos_valid = std::isfinite(_new_x) && std::isfinite(_new_y) && std::isfinite(_new_yaw);
    const bool is_valid = _landing_planner.isActive() && is_pos_valid;
    if (is_valid) {
        mavlink_trajectory_representation_waypoints_t wp_message;
        mavlink_msg_trajectory_representation_waypoints_decode(&_message, &wp_message);

        const float d_x = abs(_current_pos_x - _new_x);
        const float d_y = abs(_current_pos_y - _new_y);

        std::cout << "[" << _current_pos_x << ", " << _current_pos_y << "] >>->> "
                  << "[" << _new_x << ", " << _new_y << "]"
                  << "  d_x=" << d_x << " d_y" << d_y << std::endl;

        const float vel_scale = 0.5f;  // TODO make this ROS parameter

        Eigen::Vector2f vel_new(_current_pos_x - _new_x, _current_pos_y - _new_y);
        vel_new = vel_new.normalized() * vel_scale;

        static Eigen::Vector2f vel(vel_new);

        const float smoothing_factor = 0.9f;  // TODO make this ROS parameter
        const float inv_smoothing_factor = 1.f - smoothing_factor;

        vel = vel * smoothing_factor + vel_new * inv_smoothing_factor;  // smoothing

        wp_message.valid_points = 1;
        wp_message.pos_x[0] = NAN;
        wp_message.pos_y[0] = NAN;
        wp_message.pos_z[0] = -_landing_planner.getSearchAltitude();
        wp_message.pos_yaw[0] = _new_yaw;
        wp_message.vel_x[0] = -vel.x();
        wp_message.vel_y[0] = -vel.y();
        wp_message.vel_z[0] = NAN;

        mavlink_message_t corrected_traj_message;
        mavlink_msg_trajectory_representation_waypoints_encode(1, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &corrected_traj_message,
                                                               &wp_message);
        _mavlink_passthrough->send_message(corrected_traj_message);
    } else {
        mavsdk::MissionRaw::MissionProgress progress = _mission_raw->mission_progress();

        const bool is_in_mission = _flight_mode == mavsdk::Telemetry::FlightMode::Mission;
        const bool is_curr_landing_state_on_ground = _landed_state == mavsdk::Telemetry::LandedState::OnGround;
        const bool is_prev_landing_state_landing = _previous_landed_state == mavsdk::Telemetry::LandedState::Landing;
        const bool is_mission_over = progress.current >= progress.total-1; // -1 needed, as MAVSDK not always counts correctly
        const bool is_safe_landing_ended = _landing_planner.state() == landing_planner::LandingSearchState::ENDED;
        const bool is_on_ground_after_mission_with_safe_landing = is_in_mission && is_curr_landing_state_on_ground &&
                                                                  is_prev_landing_state_landing && is_mission_over &&
                                                                  is_safe_landing_ended;

        if (!is_on_ground_after_mission_with_safe_landing) {
            mavlink_trajectory_representation_waypoints_t wp_message;
            mavlink_msg_trajectory_representation_waypoints_decode(&_message, &wp_message);

            mavlink_message_t forwarded_traj_message;
            mavlink_msg_trajectory_representation_waypoints_encode(1, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &forwarded_traj_message,
                                                               &wp_message);
            _mavlink_passthrough->send_message(forwarded_traj_message);
        } else {
            // If safe landing kicks in during the final landing of a mission, the mode does not automatically switch to
            // Hold once on th ground. We must manually trigger the mode change in order to complete the mission and
            // trigger a disarm.
            std::cout << std::string(missionManagerOut)
                      << "Landed at end of mission with Safe Landing. Issuing Hold command..." << std::endl;
            std::function<void(mavsdk::Action::Result)> hold_callback = [](mavsdk::Action::Result result) {
                std::cout << "Hold command issued with result " << result << std::endl;
            };
            _action->hold_async(hold_callback);
        }
    }
    _frequency_traj.tic();

    // mavsdk::MissionRaw::MissionProgress progress = _mission_raw->mission_progress();
    // std::cout << "OA TEST :: "
    //           << " flight mode='" << _flight_mode << "'"
    //           << " landed state='" << _landed_state << "'"
    //           << " prev landed state='" << _previous_landed_state << "'"
    //           << " mission prog " << progress.current << "/" << progress.total << std::endl;
    // std::cout << "SL planner state = " << _landing_planner.state() << std::endl;
}

void MissionManager::set_global_position_reference() {
    while (!int_signal) {
        std::string status{};
        // Get global origin to set the reference global position
        auto cmd_result = _telemetry->get_gps_global_origin();

        if (cmd_result.first == mavsdk::Telemetry::Result::Success) {
            if (!_global_origin_reference_set) {
                std::cout << std::string(missionManagerOut)
                          << "Successfully received a GPS_GLOBAL_ORIGIN MAVLink message"
                          << std::endl;
            }

            const bool lat_valid = cmd_result.second.latitude_deg != 0.0;
            const bool lon_valid = cmd_result.second.longitude_deg != 0.0;

            if (lat_valid && lon_valid) {
                const bool lat_changed = _ref_latitude.load() != cmd_result.second.latitude_deg;
                const bool lon_changed = _ref_longitude.load() != cmd_result.second.longitude_deg;
                const bool alt_changed = _ref_altitude.load() != cmd_result.second.altitude_m;

                const bool ori_changed = lat_changed || lon_changed || alt_changed;

                if ( ori_changed ) {
                    _ref_latitude.store(cmd_result.second.latitude_deg);
                    _ref_longitude.store(cmd_result.second.longitude_deg);
                    _ref_altitude.store(cmd_result.second.altitude_m);

                    status = std::string(missionManagerOut) +
                             "Global position reference set: Latitude: " + std::to_string(_ref_latitude) +
                             " deg | Longitude: " + std::to_string(_ref_longitude) +
                             " deg | Altitude (AMSL): " + std::to_string(_ref_altitude) + " meters";
                    _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);

                    _global_origin_reference_set = true;
                }
            } else {
                if (!_global_origin_reference_set) {
                    status = std::string(missionManagerOut) +
                             "Failed to set the global origin reference because the received values are invalid.";
                }
            }

        } else if (cmd_result.first == mavsdk::Telemetry::Result::Timeout) {
            status = std::string(missionManagerOut) +
                     "GPS_GLOBAL_ORIGIN stream request timeout. Message not yet received. Retrying...";
        } else {
            status = std::string(missionManagerOut) +
                     "GPS_GLOBAL_ORIGIN stream request failed. Retrying...";
        }

        if ( status != "") {
            std::cout << status << std::endl;
        }
        std::this_thread::sleep_for(1s);
    }

}

mavsdk::geometry::CoordinateTransformation::LocalCoordinate MissionManager::get_local_position_from_local_offset(
    const double& offset_x, const double& offset_y) const {
    // Given an offset in body-frame x and y, compute the local position
    const double local_position_x =
        (std::cos(_current_yaw) * offset_x - std::sin(_current_yaw) * offset_y) + _current_pos_x;
    const double local_position_y =
        (std::sin(_current_yaw) * offset_x + std::cos(_current_yaw) * offset_y) + _current_pos_y;
    return mavsdk::geometry::CoordinateTransformation::LocalCoordinate{local_position_x, local_position_y};
}

mavsdk::geometry::CoordinateTransformation::GlobalCoordinate MissionManager::get_global_position_from_local_position(
    mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_position) const {
    const mavsdk::geometry::CoordinateTransformation ct({_ref_latitude, _ref_longitude});
    return ct.global_from_local(local_position);
}

mavsdk::geometry::CoordinateTransformation::GlobalCoordinate MissionManager::get_global_position_from_local_offset(
    const double& offset_x, const double& offset_y) const {
    const mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_position =
        get_local_position_from_local_offset(offset_x, offset_y);
    return get_global_position_from_local_position(local_position);
}

void MissionManager::set_new_waypoint(const double& lat, const double& lon, const double& alt_amsl) {
    _new_latitude = lat;
    _new_longitude = lon;
    _new_altitude_amsl = alt_amsl;
}
void MissionManager::set_new_local_waypoint(const double& x, const double& y, const double& yaw) {
    _new_x = x;
    _new_y = y;
    _new_yaw = yaw;
}

bool MissionManager::arrived_to_new_waypoint() {
    if ((std::abs(_new_latitude - _current_latitude) <= 1.0E-5) &&
        (std::abs(_new_longitude - _current_longitude) <= 1.0E-5) &&
        (std::abs(_new_altitude_amsl - _current_altitude_amsl) <= 1.0)) {  // ~1 meter acceptance radius
        return true;
    }

    return false;
}

bool MissionManager::is_stationary() {
    static const double vel_tol = 0.5;
    const double vel_mag =
        std::sqrt(std::pow(_current_vel_x, 2) + std::pow(_current_vel_y, 2) + std::pow(_current_vel_z, 2));
    return debounce_is_stationary(vel_mag < vel_tol);
}

bool MissionManager::debounce_is_stationary(bool is_stationary) {
    // The purpose of the debouncing is not only to prevent multiple triggers, but also to include a delay between when
    // the vehicle stops and when it is allowed to process any actions. This gives the landing site detection
    // time to assess the ground below the stopping point.
    static const int DEBOUNCE_COUNT_REQUIRED = 10;
    if (is_stationary) {
        _is_stationary_debounce_counter++;
        if (_is_stationary_debounce_counter >= DEBOUNCE_COUNT_REQUIRED) {
            // Vehicle is confirmed to be stationary
            _is_stationary_debounce_counter = 0;
            return true;
        }
    } else {
        _is_stationary_debounce_counter = 0;
    }
    return false;
}

bool MissionManager::landing_triggered() {
    const bool manual_triggered_land = _flight_mode == mavsdk::Telemetry::FlightMode::Land;
    const bool mission_land = _flight_mode == mavsdk::Telemetry::FlightMode::Mission &&
                              _landed_state == mavsdk::Telemetry::LandedState::Landing;
    return manual_triggered_land || mission_land;
}

bool MissionManager::under_manual_control() {
    switch (_flight_mode) {
        case mavsdk::Telemetry::FlightMode::Manual:
        case mavsdk::Telemetry::FlightMode::Altctl:
        case mavsdk::Telemetry::FlightMode::Posctl:
        case mavsdk::Telemetry::FlightMode::Acro:
        case mavsdk::Telemetry::FlightMode::Stabilized:
        case mavsdk::Telemetry::FlightMode::Rattitude:
            return true;
        default:
            return false;
    }
}

void MissionManager::go_to_new_local_waypoint(
    mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_waypoint, const double altitude,
    const double yaw_rad) {
    // Convert to global position
    const mavsdk::geometry::CoordinateTransformation::GlobalCoordinate waypoint =
        get_global_position_from_local_position(local_waypoint);

    // Store info how to continue after landing site found (return to mission or land).
    if (!_was_landing_paused && !_was_mission_paused) {
        const bool manual_triggered_land = _flight_mode == mavsdk::Telemetry::FlightMode::Land;
        const bool mission_land = _flight_mode == mavsdk::Telemetry::FlightMode::Mission &&
                                  _landed_state == mavsdk::Telemetry::LandedState::Landing;

        if (manual_triggered_land) {
            _was_landing_paused = true;
            _was_mission_paused = false;
        } else if (mission_land) {
            _was_landing_paused = false;
            _was_mission_paused = true;
        }
    }

    // Store info how to continue after landing site found (return to mission or land).
    if (!_was_landing_paused && !_was_mission_paused) {
        const bool manual_triggered_land = _flight_mode == mavsdk::Telemetry::FlightMode::Land;
        const bool mission_land = _flight_mode == mavsdk::Telemetry::FlightMode::Mission &&
                                  _landed_state == mavsdk::Telemetry::LandedState::Landing;

        if (manual_triggered_land) {
            _was_landing_paused = true;
            _was_mission_paused = false;
        } else if (mission_land) {
            _was_landing_paused = false;
            _was_mission_paused = true;
        }
    }

    // Go to waypoint
    set_new_local_waypoint(local_waypoint.north_m, local_waypoint.east_m, _current_yaw);
    std::cout << "set wp to "
              << "[" << _new_x << ", " << _new_y << "]" << std::endl;
}

void MissionManager::handle_safe_landing(std::chrono::time_point<std::chrono::system_clock> now) {
    std::unique_lock<std::mutex> lock(mission_manager_config_mtx);
    const bool safe_landing_enabled = _mission_manager_config.safe_landing_enabled;
    const float safe_landing_distance_to_ground = _mission_manager_config.safe_landing_distance_to_ground;
    const bool safe_landing_try_landing_after_action = _mission_manager_config.safe_landing_try_landing_after_action;
    const std::string safe_landing_on_no_safe_land = _mission_manager_config.safe_landing_on_no_safe_land;
    const double landing_site_search_max_speed = _mission_manager_config.landing_site_search_max_speed;
    const double landing_site_search_max_distance = _mission_manager_config.landing_site_search_max_distance;
    const double landing_site_search_min_height = _mission_manager_config.landing_site_search_min_height;
    const double landing_site_search_min_distance_after_abort =
        _mission_manager_config.landing_site_search_min_distance_after_abort;
    const double landing_site_search_arrival_radius = _mission_manager_config.landing_site_search_arrival_radius;
    const double landing_site_search_assess_time = _mission_manager_config.landing_site_search_assess_time;
    const std::string landing_site_search_strategy = _mission_manager_config.landing_site_search_strategy;
    const double landing_site_search_spiral_spacing = _mission_manager_config.landing_site_search_spiral_spacing;
    const int landing_site_search_spiral_points = _mission_manager_config.landing_site_search_spiral_points;
    const double global_position_waypoint_lat = _mission_manager_config.global_position_waypoint_lat;
    const double global_position_waypoint_lon = _mission_manager_config.global_position_waypoint_lon;
    const double global_position_waypoint_alt_amsl = _mission_manager_config.global_position_waypoint_alt_amsl;
    const double local_position_offset_x = _mission_manager_config.local_position_offset_x;
    const double local_position_offset_y = _mission_manager_config.local_position_offset_y;
    const double local_position_offset_z = _mission_manager_config.local_position_offset_z;
    lock.unlock();

    const uint8_t safe_landing_state = _landing_condition_state_update_callback();
    const float height_above_obstacle = _height_above_obstacle_update_callback();

    mavsdk::MissionRaw::MissionProgress progress = _mission_raw->mission_progress();

    if (safe_landing_enabled) {
        const auto ros_now = this->get_clock()->now();
        const auto s_since_last_traj = (ros_now - _time_last_traj).seconds();
        const bool get_traj = s_since_last_traj < 0.5f;

        if (_got_traj && !get_traj) {
            _got_traj = false;
        } else if (get_traj) {
            _got_traj = true;
        }

        mavlink_message_t new_heartbeat_message;
        mavlink_heartbeat_t heartbeat;
        heartbeat.system_status = MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_encode(1, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &new_heartbeat_message, &heartbeat);
        _mavlink_passthrough->send_message(new_heartbeat_message);

        /*
         * If we're not handling an action already, check if we need to start one.
         */
        if (!_action_triggered) {
            std::string status{};

            if (landing_triggered()) {
                if (height_above_obstacle > 1.5 && safe_landing_state == 0 /*eLandingMapperState::UNHEALTHY*/) {
                    // If the safe landing status is unhealthy, then hold position.
                    _action->hold();

                    status = std::string(missionManagerOut) + "Safe landing system not healthy. Holding position...";
                    _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                    std::cout << status << std::endl;

                    _action_triggered = true;
                    _last_time = now;

                } else if (height_above_obstacle > 1.5 && safe_landing_state == 1 /*eLandingMapperState::UNKNOWN*/) {
                    // If the vehicle is bellow the defined maximum distance to ground to determine if it is
                    // safe to land or not, then it will still try to land until it reaches an height that
                    // allows it to determine if it can land or not. Otherwise, it holds position.
                    _action->hold();

                    status =
                        std::string(missionManagerOut) + "Cannot determine if it is safe to land. Holding position...";
                    _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                    std::cout << status << std::endl;

                    _action_triggered = true;
                    _last_time = now;

                } else if (safe_landing_state == 4 /*eLandingMapperState::CAN_NOT_LAND*/) {
                    std::cout << std::string(missionManagerOut) << "Cannot land! -----------------------" << std::endl;
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

                    } else if (safe_landing_on_no_safe_land == "LANDING_SITE_SEARCH") {
                        if (_global_origin_reference_set && _got_traj ) {
                            std::cout << "*" << std::endl
                                      << "***" << std::endl
                                      << "***** Starting Landing Site Search" << std::endl
                                      << "***" << std::endl
                                      << "*" << std::endl;

                            status = "Starting Landing Site Search";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);

                            // Configure landing planner
                            landing_planner::LandingPlannerConfig lp_config;
                            lp_config.max_distance = landing_site_search_max_distance;
                            lp_config.min_height = landing_site_search_min_height;
                            lp_config.min_distance_after_abort = landing_site_search_min_distance_after_abort;
                            lp_config.waypoint_arrival_radius = landing_site_search_arrival_radius;
                            lp_config.site_assess_time = landing_site_search_assess_time;
                            lp_config.search_strategy = landing_site_search_strategy;
                            lp_config.spiral_search_spacing = landing_site_search_spiral_spacing;
                            lp_config.spiral_search_points = landing_site_search_spiral_points;

                            // Start search
                            _landing_planner.startSearch(_current_pos_x, _current_pos_y, _current_yaw, -_current_pos_z,
                                                         lp_config);

                            // set landing waypoint to check in case landing detection is not reported over MAVSDK
                            _landing_waypoint_id = progress.current;

                            if (_landing_planner.isActive()) {
                                // Lower the maximum speed
                                mavsdk::Action::Result result =
                                    _action->set_maximum_speed(landing_site_search_max_speed);
                                if (result == mavsdk::Action::Result::Success) {
                                    std::cout << std::string(missionManagerOut) << "Lowering maximum speed from "
                                              << _original_max_speed << " to " << landing_site_search_max_speed
                                              << std::endl;
                                } else {
                                    std::cout << std::string(missionManagerOut)
                                              << "Could not lower maximum speed for landing site search." << std::endl;
                                }

                                // Set the first waypoint in the search pattern
                                const mavsdk::geometry::CoordinateTransformation::LocalCoordinate new_wpt =
                                    _landing_planner.getCurrentWaypoint();
                                go_to_new_local_waypoint(new_wpt, _landing_planner.getSearchAltitude(),
                                                         _current_yaw * 180. / M_PI);

                                // std::stringstream ss;
                                // ss << missionManagerOut << "Landing site search started
                                // at: Lat " << std::fixed
                                //     << std::setprecision(6) << _new_latitude << "°, Lon "
                                //     << _new_longitude << "°, Alt "
                                //     << std::setprecision(2) << _new_altitude_amsl << "m
                                //     AMSL, Local (" << new_wpt.north_m
                                //     << ", " << new_wpt.east_m << ")" << std::endl;
                                // status = ss.str();
                                // _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info,
                                // status);
                            } else {
                                // Planner did not start correctly.
                                _action->hold();
                                landing_site_search_has_ended("NSC");

                                status = std::string(missionManagerOut) +
                                         "Landing planner could not start. Holding position...";
                                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning,
                                                                  status);
                            }
                        } else if (!_global_origin_reference_set) {
                            // Planner did not start correctly.
                            _action->hold();
                            landing_site_search_has_ended("No GO");

                            status = std::string(missionManagerOut) +
                                     "Global position reference not set. Holding position...";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Error,
                                                              status);
                        }
                        else if (!_got_traj) {
                            _action->hold();
                            landing_site_search_has_ended("No OA");

                            status = std::string(missionManagerOut) +
                                     "Landing planner could not start. No OA active in PX4. Holding position...";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Error,
                                                              status);
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

        /*
         *  Process any active actions
         */
        if (_landing_planner.isActive()) {
            const bool on_ground =
                _landed_state == mavsdk::Telemetry::LandedState::OnGround ||
                _landed_state ==
                    mavsdk::Telemetry::LandedState::TakingOff;  // Necessary for mission mode, as "on ground" is not
                                                                // always communicated when Land WP is inside a mission
            const bool manual_control = under_manual_control();
            const bool rtl_active = _flight_mode == mavsdk::Telemetry::FlightMode::ReturnToLaunch;
            const bool wp_switched = (_landing_waypoint_id != -1) && (_landing_waypoint_id < progress.current);

            const bool end_landing_site_search = on_ground || manual_control || rtl_active || wp_switched || !_got_traj;

            if (end_landing_site_search) {
                _landing_planner.endSearch();

                std::string debug_info = "";
                if (on_ground) {
                    std::cout << std::string(missionManagerOut) << "Vehicle landed or approaching ground." << std::endl;
                    debug_info = "OG";
                } else if (manual_control) {
                    std::cout << std::string(missionManagerOut) << "Pilot has taken manual control (" << _flight_mode
                              << " mode). Canceling safe landing." << std::endl;
                    debug_info = "MAN";
                } else if (rtl_active) {
                    std::cout << std::string(missionManagerOut) << "RTL triggered. Cancelling safe landing."
                              << std::endl;
                    debug_info = "RTL";
                } else if (!_got_traj) {
                    std::cout << std::string(missionManagerOut) << "OA not active on PX4. Cancelling safe landing."
                              << std::endl;
                    debug_info = "No OA";
                } else if (wp_switched) {
                    std::cout << std::string(missionManagerOut) << "Target WP changed from " << _landing_waypoint_id
                              << " to " << progress.current << ". Ending safe landing search." << std::endl;
                    debug_info = "WPC";
                }

                landing_site_search_has_ended(debug_info);
            } else {
                update_landing_site_search(safe_landing_state, height_above_obstacle,
                                           safe_landing_try_landing_after_action);
            }
        } else if (std::isfinite(_new_latitude) && std::isfinite(_new_longitude) && std::isfinite(_new_altitude_amsl)) {
            if (arrived_to_new_waypoint()) {
                std::cout << "[DEBUG] " << __FUNCTION__ << ":" << __LINE__ << std::endl;
                // if the user set it wants to try landing again after arriving to the waypoint
                // then land and unset the new waypoint
                if (safe_landing_try_landing_after_action) {
                    std::cout << "[DEBUG] " << __FUNCTION__ << ":" << __LINE__ << std::endl;
                    _action->land();
                    set_new_waypoint(NAN, NAN, NAN);
                }
            }
        }
    }
}

void MissionManager::change_mission_wp_location(mavsdk::MissionRaw::MissionItem& _wp, int32_t _x, int32_t _y,
                                                float _z) const {
    _wp.x = _x;
    _wp.y = _y;
    _wp.z = _z;
}

void MissionManager::change_missions_landing_site_to_current(const mavsdk::MissionRaw::MissionProgress& _progress) {
    const mavsdk::geometry::CoordinateTransformation::GlobalCoordinate global_waypoint =
        get_global_position_from_local_position({_current_pos_x, _current_pos_y});

    mavsdk::MissionRaw::Result result;
    std::vector<mavsdk::MissionRaw::MissionItem> mission;
    _mission_raw->pause_mission();
    std::tie(result, mission) = _mission_raw->download_mission();

    // Hack for last wp is landing. Unclear why necessary.
    int id = _progress.current - 1;
    _landing_waypoint_id = _progress.current;

    mavsdk::MissionRaw::MissionItem* landing_wp = &mission[id];
    if (landing_wp->command != 21 /* MAV_CMD_NAV_TAKEOFF */) {
        std::cout << "**************** " << __LINE__ << std::endl;
        std::cout << "*    WARNING   * " << __LINE__ << std::endl;
        std::cout << "**************** " << __LINE__ << std::endl;
        std::cout << "Mission item no landing waypoint" << std::endl;
        std::cout << *landing_wp << std::endl;
        landing_wp = &mission[id + 1];

        if (landing_wp->command != 21 /* MAV_CMD_NAV_TAKEOFF */) {
            std::cout << "**************** " << __LINE__ << std::endl;
            std::cout << "*    WARNING   * " << __LINE__ << std::endl;
            std::cout << "**************** " << __LINE__ << std::endl;
            std::cout << "Mission item no landing waypoint" << std::endl;
            std::cout << *landing_wp << std::endl;
        }
    }

    _landing_latitude_deg = landing_wp->x;
    _landing_longitude_deg = landing_wp->y;
    _landing_altitude_m = landing_wp->z;

    change_mission_wp_location(*landing_wp, global_waypoint.latitude_deg * 10e6, global_waypoint.longitude_deg * 10e6,
                               -_current_pos_z);

    mavsdk::MissionRaw::Result res_upload = _mission_raw->upload_mission(mission);
    mavsdk::MissionRaw::Result res_start = _mission_raw->start_mission();

    const bool upload_failed = res_upload != mavsdk::MissionRaw::Result::Success;
    const bool start_failed = res_start != mavsdk::MissionRaw::Result::Success;
    if (upload_failed || start_failed) {
        std::cout << "failed mission (" << id << "/" << _progress.current << "/" << _progress.total << ")" << std::endl;
        for (const auto& item : mission) {
            std::cout << "- " << item << std::endl;
        }
    }
}

void MissionManager::restore_missions_landing_site_to_current(const std::string& _debug) {
    mavsdk::MissionRaw::Result result;
    std::vector<mavsdk::MissionRaw::MissionItem> mission;
    std::tie(result, mission) = _mission_raw->download_mission();
    mavsdk::MissionRaw::MissionProgress progress = _mission_raw->mission_progress();

//    std::cout << "mission (" << progress.current << "/" << progress.total << ")" << std::endl;
//    for (const auto& item : mission) {
//        std::cout << "- " << item << std::endl;
//    }
//    std::cout << std::endl;

    int id = progress.current - 1;
    if (progress.current + 1 == progress.total) {
        id = progress.current;
    }
    if (_debug == "WPC") {
        id--;
    }

    mavsdk::MissionRaw::MissionItem* landing_wp = &mission[id];
    if (landing_wp->command != 21) {
        std::cout << "**************** " << __LINE__ << std::endl;
        std::cout << "*    WARNING   * " << __LINE__ << std::endl;
        std::cout << "**************** " << __LINE__ << std::endl;
        std::cout << "Mission item no landing waypoint" << std::endl;
        std::cout << landing_wp << std::endl;
        landing_wp = &mission[id + 1];
    }

//    std::cout << "Reset" << std::endl;
//    std::cout << *landing_wp << std::endl;

    change_mission_wp_location(*landing_wp, _landing_latitude_deg, _landing_longitude_deg, _landing_altitude_m);

//    std::cout << "to" << std::endl;
//    std::cout << *landing_wp << std::endl;
//
//    std::cout << "mission now (" << progress.current << "/" << progress.total << ")" << std::endl;
//    for (const auto& item : mission) {
//        std::cout << "- " << item << std::endl;
//    }

    _mission_raw->upload_mission(mission);

    _landing_latitude_deg = NAN;
    _landing_longitude_deg = NAN;
    _landing_altitude_m = NAN;
}

void MissionManager::update_landing_site_search(const uint8_t safe_landing_state, const float height_above_obstacle,
                                                const bool land_when_found_site) {
    /*
     *  STEP 1: Decide what to do
     */
    bool should_initiate_landing = false;
    const landing_planner::LandingSearchState search_state = _landing_planner.state();
    if (search_state == landing_planner::LandingSearchState::SEARCH_ACTIVE &&
        safe_landing_state == 3 /*eLandingMapperState::CAN_LAND*/ &&
        _landing_planner.isValidCandidateSite({_current_pos_x, _current_pos_y})) {
        // Observed a safe place while searching for a landing site.
        _landing_planner.candidateSiteFoundAt({_current_pos_x, _current_pos_y});
    } else if (search_state == landing_planner::LandingSearchState::ATTEMPTING_TO_LAND) {
        if (safe_landing_state == 0 /*eLandingMapperState::UNHEALTHY*/ ||
            safe_landing_state == 1 /*eLandingMapperState::UNKNOWN*/ ||
            safe_landing_state == 4 /*eLandingMapperState::CAN_NOT_LAND*/) {
            // Attempting to land, but there's a problem.
            std::string status = std::string(missionManagerOut) + "Aborting landing at candidate site";
            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
            std::cout << status << std::endl;
            _landing_planner.abortLanding(-_current_pos_z, height_above_obstacle);

            // Debug output
            std::cout << "_landing_waypoint_id " << _landing_waypoint_id << std::endl;
            std::cout << "safe_landing_state " << (int)safe_landing_state << std::endl;
        }
    } else if (safe_landing_state == 5 /*eLandingMapperState::TOO_HIGH*/) {
        // Adjust search altitude
        _landing_planner.adjustSearchAltitude(-_current_pos_z, height_above_obstacle,
                                              _mission_manager_config.safe_landing_distance_to_ground);
    } else {
        _landing_planner.checkForWaypointArrival({_current_pos_x, _current_pos_y}, -_current_pos_z, is_stationary(),
                                                 safe_landing_state);
        if (_landing_planner.state() == landing_planner::LandingSearchState::ATTEMPTING_TO_LAND) {
            // Switched into ATTEMPTING_TO_LAND
            should_initiate_landing = true;
        }
    }

    mavsdk::MissionRaw::MissionProgress progress = _mission_raw->mission_progress();
    // std::cout << "landing planner state=" << _landing_planner.state()
    //             << "  safe_landing_state=" <<  (landing_mapper::eLandingMapperState)safe_landing_state
    //             << "  should_initiate_landing=" <<  should_initiate_landing
    //             << "  wp updated=" << _landing_planner.waypointUpdated()
    //             << "  wp id=" <<  _landing_waypoint_id
    //             << "  mission prog " <<  progress.current
    //             << "/" <<  progress.total
    //             << std::endl;

    /*
     *  STEP 2: Send commands to vehicle
     */
    std::string status = std::string(missionManagerOut);
    if (should_initiate_landing) {
        status += "Landing site found. ";
        set_new_local_waypoint(NAN, NAN, NAN);
        if (land_when_found_site) {
            status += "Landing...";

            // TODO rework and put to a separate function
            if (_was_mission_paused) {
                status += " from mission ";

                change_missions_landing_site_to_current(progress);
            } else if (_was_landing_paused) {
                status += " from manual ";
                _action->hold();  // Quickly switching to hold mode is necessary in order to make PX4 take the new xy
                                  // coordinate as landing position
                std::this_thread::sleep_for(200ms);
                _action->land();
            } else {
                status += " 3 ";
                // TODO check why this happens and make something reasonable here.
            }
        } else {
            status += "Holding position...";
            _action->hold();
            // End the search
            _landing_planner.endSearch();
        }
        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
    } else if (_landing_planner.state() == landing_planner::LandingSearchState::ENDED) {
        // End of search pattern.
        // Hold position.
        status += "End of landing site search. Holding position...";
        set_new_local_waypoint(NAN, NAN, NAN);
        mavsdk::Action::Result result = _action->hold();
        if (result == mavsdk::Action::Result::Success) {
            std::cout << std::string(missionManagerOut) << "Switched to HOLD mode." << std::endl;
            _flight_mode = mavsdk::Telemetry::FlightMode::Hold;
        } else {
            std::cout << std::string(missionManagerOut) << "Could not switched to HOLD mode." << std::endl;
        }
        std::this_thread::sleep_for(500ms);
        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
    } else if (_landing_planner.waypointUpdated()) {
        // Command new waypoint
        const mavsdk::geometry::CoordinateTransformation::LocalCoordinate new_wpt =
            _landing_planner.getCurrentWaypoint();
        go_to_new_local_waypoint(new_wpt, _landing_planner.getSearchAltitude(), _current_yaw * 180. / M_PI);
        std::stringstream ss;
        ss << "[Landing Site Search] Waypoint set: Local (" << new_wpt.north_m << ", " << new_wpt.east_m << ")";
        status += ss.str();
        // _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
    } else {
        status += "No change in behaviour";
        // No change in behaviour
        return;
    }
    std::cout << status << std::endl;

    /*
     *  STEP 3: Restore normal flight configuration if search has ended
     */
    if (_landing_planner.state() == landing_planner::LandingSearchState::ENDED) {
        landing_site_search_has_ended("END");
    }
}

void MissionManager::landing_site_search_has_ended(const std::string& _debug) {
    mavsdk::Action::Result result = _action->set_maximum_speed(_original_max_speed);
    if (result == mavsdk::Action::Result::Success) {
        std::cout << std::string(missionManagerOut) << "Restoring maximum speed to " << _original_max_speed
                  << std::endl;
    } else {
        std::cout << std::string(missionManagerOut) << "Could not restore maximum speed after landing site search."
                  << std::endl;
    }

    _action_triggered = false;
    std::cout << "    *" << std::endl
              << "  ***" << std::endl
              << "***** Landing Site Search has ended" << std::endl
              << "  ***" << std::endl
              << "    *" << std::endl;

    if (_server_utility) {
        std::stringstream ss;
        ss << "Landing Site Search has ended";
        if (_debug != "") {
            ss << " (" << _debug << ")";
        }
        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, ss.str());
    }

    // TODO rework and put to a separate function
    if (_was_mission_paused) {
        if (std::isfinite(_landing_latitude_deg) && std::isfinite(_landing_longitude_deg)) {
            restore_missions_landing_site_to_current(_debug);
        }

        _landing_waypoint_id = -1;
    }

    _was_mission_paused = false;
    _was_landing_paused = false;
}

void MissionManager::handle_simple_collision_avoidance(std::chrono::time_point<std::chrono::system_clock> now) {
    if (_mission_manager_config.simple_collision_avoid_enabled != 0U) {
        const bool in_air = (_landed_state == mavsdk::Telemetry::LandedState::InAir);
        // std::cout << "Depth measured: " << _distance_to_obstacle_update_callback()
        //           << " | threshold: " << _mission_manager_config.simple_collision_avoid_distance_threshold
        //           << " | in air: " << in_air << " | is action triggered? " << std::boolalpha
        //           << _action_triggered << std::endl;

        if (std::isfinite(_distance_to_obstacle_update_callback()) &&
            _distance_to_obstacle_update_callback() <=
                _mission_manager_config.simple_collision_avoid_distance_threshold &&
            in_air && !_action_triggered) {  // only trigger the condition when the vehicle is in-air
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

    // Get local position and velocity
    _telemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed position_velocity) {
        _current_pos_x = position_velocity.position.north_m;
        _current_pos_y = position_velocity.position.east_m;
        _current_pos_z = position_velocity.position.down_m;
        _current_vel_x = position_velocity.velocity.north_m_s;
        _current_vel_y = position_velocity.velocity.east_m_s;
        _current_vel_z = position_velocity.velocity.down_m_s;
    });

    // Get the original maximum speed when not doing a landing site search
    _original_max_speed = _action->get_maximum_speed().second;
    std::cout << "Original maximum speed = " << _original_max_speed << std::endl;

    // Get yaw
    _telemetry->subscribe_attitude_euler(
        [this](mavsdk::Telemetry::EulerAngle euler_angle) { _current_yaw = euler_angle.yaw_deg * M_PI / 180.0; });

    // Get the flight mode
    _telemetry->subscribe_flight_mode(
        [this](mavsdk::Telemetry::FlightMode flight_mode) { _flight_mode = flight_mode; });

    // Get the landing state so we know when the vehicle is in-air, landing or on-ground
    _telemetry->subscribe_landed_state(
        [this](mavsdk::Telemetry::LandedState landed_state) {
            if ( landed_state != _landed_state) {
                _previous_landed_state = _landed_state.load();
                _landed_state = landed_state;
            }
        }
    );

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

            // After an action is triggered, we give it 5 seconds to process it before retrying.
            // Except the landing site search, which must terminate itself.
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_time).count() >= 5000 &&
                !_landing_planner.isActive()) {
                _action_triggered = false;
            }
        }

        // Decision maker runs at 20hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
