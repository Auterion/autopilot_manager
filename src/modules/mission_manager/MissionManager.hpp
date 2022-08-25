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
 * @file MissionManager.hpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#pragma once

#include <CustomActionHandler.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <ModuleBase.hpp>
#include <atomic>
#include <future>
#include <iostream>
#include <landing_planner/LandingPlanner.hpp>
#include <string>
#include <timing_tools/timing_tools.h>

// MAVSDK dependencies
#include <mavsdk/geometry.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/custom_action/custom_action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/server_utility/server_utility.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

static constexpr auto missionManagerOut = "[Mission Manager] ";

class MissionManager : public rclcpp::Node, ModuleBase {
   public:
    MissionManager(std::shared_ptr<mavsdk::System> mavsdk_system, mavsdk::Mavsdk* mavsdk,
                   const std::string& path_to_custom_action_file);
    ~MissionManager();
    MissionManager(const MissionManager&) = delete;
    auto operator=(const MissionManager&) -> const MissionManager& = delete;

    void init() override;
    void deinit() override;
    void run() override;

    struct MissionManagerConfiguration {
        uint8_t autopilot_manager_enabled = 0U;
        std::string decision_maker_input_type = "";

        std::string script_to_call = "";
        std::string api_call = "";
        double local_position_offset_x = 0.0;
        double local_position_offset_y = 0.0;
        double local_position_offset_z = 0.0;
        double local_position_waypoint_x = 0.0;
        double local_position_waypoint_y = 0.0;
        double local_position_waypoint_z = 0.0;
        double global_position_offset_lat = 0.0;
        double global_position_offset_lon = 0.0;
        double global_position_offset_alt_amsl = 0.0;
        double global_position_waypoint_lat = 0.0;
        double global_position_waypoint_lon = 0.0;
        double global_position_waypoint_alt_amsl = 0.0;

        // // Depth camera configuration
        // double camera_offset_x = 0.0;
        // double camera_offset_y = 0.0;
        // double camera_yaw = 0.0;

        uint8_t safe_landing_enabled = 0U;
        double safe_landing_distance_to_ground = 0.0;
        std::string safe_landing_on_no_safe_land = "";
        uint8_t safe_landing_try_landing_after_action = 0U;

        // Landing site search config
        double landing_site_search_max_speed = 0.0;
        double landing_site_search_max_distance = 0.0;
        double landing_site_search_min_height = 0.0;
        double landing_site_search_min_distance_after_abort = 0.0;
        double landing_site_search_arrival_radius = 0.0;
        double landing_site_search_assess_time = 0.0;
        std::string landing_site_search_strategy = "";

        // Spiral search strategy config
        double landing_site_search_spiral_spacing = 0.0;
        int landing_site_search_spiral_points = 0;

        uint8_t simple_collision_avoid_enabled = 0U;
        double simple_collision_avoid_distance_threshold = 0.0;
        std::string simple_collision_avoid_action_on_condition_true = "";
    };

    void setConfigUpdateCallback(std::function<MissionManagerConfiguration()> callback) {
        _config_update_callback = callback;
    }

    void getDistanceToObstacleCallback(std::function<float()> callback) {
        _distance_to_obstacle_update_callback = callback;
    }

    void getCanLandStateCallback(std::function<uint8_t()> callback) {
        _landing_condition_state_update_callback = callback;
    }

    void getCanLandAtPositionStateCallback(std::function<uint8_t(float x, float y)> callback) {
        _landing_planner.setLandingStateAtPositionCallback(callback);
    }

    void getHeightAboveObstacleCallback(std::function<float()> callback) {
        _height_above_obstacle_update_callback = callback;
    }

    void decision_maker_run();

   private:
    void handle_safe_landing(std::chrono::time_point<std::chrono::system_clock> now);
    void handle_simple_collision_avoidance(std::chrono::time_point<std::chrono::system_clock> now);

    void update_landing_site_search(const uint8_t safe_landing_state, const float height_above_obstacle,
                                    const bool land_when_found_site);
    void landing_site_search_has_ended(const std::string& _debug = "");

    void on_mavlink_trajectory_message(const mavlink_message_t& _message);

    void set_global_position_reference();
    void set_new_waypoint(const double& lat, const double& lon, const double& alt_amsl);
    void set_new_local_waypoint(const double& x, const double& y, const double& yaw);
    bool arrived_to_new_waypoint();
    bool is_stationary();
    bool debounce_is_stationary(bool is_stationary);

    bool landing_triggered();
    bool under_manual_control();

    void go_to_new_local_waypoint(mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_waypoint,
                                  const double altitude, const double yaw_rad);

    mavsdk::geometry::CoordinateTransformation::LocalCoordinate get_local_position_from_local_offset(
        const double& offset_x, const double& offset_y) const;
    mavsdk::geometry::CoordinateTransformation::GlobalCoordinate get_global_position_from_local_position(
        mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_position) const;
    mavsdk::geometry::CoordinateTransformation::GlobalCoordinate get_global_position_from_local_offset(
        const double& offset_x, const double& offset_y) const;

    void change_mission_wp_location(mavsdk::MissionRaw::MissionItem& _wp, int32_t _x, int32_t _y, float _z) const;
    void change_missions_landing_site_to_current(const mavsdk::MissionRaw::MissionProgress& _progress);
    void restore_missions_landing_site_to_current(const std::string& _debug);

    void send_corrected_trajectory();

    std::function<MissionManagerConfiguration()> _config_update_callback;
    std::function<float()> _distance_to_obstacle_update_callback;
    std::function<uint8_t()> _landing_condition_state_update_callback;
    std::function<float()> _height_above_obstacle_update_callback;

    std::string _path_to_custom_action_file;

    MissionManagerConfiguration _mission_manager_config;

    std::mutex mission_manager_config_mtx;

    std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
    std::shared_ptr<mavsdk::System> _mavsdk_system;
    std::shared_ptr<CustomActionHandler> _custom_action_handler;
    std::shared_ptr<mavsdk::Action> _action;
    std::shared_ptr<mavsdk::MissionRaw> _mission_raw;
    std::shared_ptr<mavsdk::Telemetry> _telemetry;
    std::shared_ptr<mavsdk::ServerUtility> _server_utility;
    std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;

    std::atomic<bool> _action_triggered;
    std::atomic<mavsdk::Telemetry::FlightMode> _flight_mode{mavsdk::Telemetry::FlightMode::Unknown};
    std::atomic<mavsdk::Telemetry::LandedState> _landed_state{mavsdk::Telemetry::LandedState::Unknown};
    std::atomic<mavsdk::Telemetry::LandedState> _previous_landed_state{mavsdk::Telemetry::LandedState::Unknown};
    std::atomic<bool> _is_global_position_ok;
    std::atomic<bool> _is_home_position_ok;
    std::atomic<bool> _get_gps_origin_success;
    std::atomic<bool> _global_origin_reference_set;

    std::atomic<int> _is_stationary_debounce_counter;

    std::atomic<double> _current_latitude;
    std::atomic<double> _current_longitude;
    std::atomic<double> _current_altitude_amsl;
    std::atomic<double> _ref_latitude;
    std::atomic<double> _ref_longitude;
    std::atomic<double> _ref_altitude;
    std::atomic<double> _current_pos_x;
    std::atomic<double> _current_pos_y;
    std::atomic<double> _current_pos_z;
    std::atomic<double> _current_vel_x;
    std::atomic<double> _current_vel_y;
    std::atomic<double> _current_vel_z;
    std::atomic<double> _current_yaw;
    std::atomic<double> _new_latitude;
    std::atomic<double> _new_longitude;
    std::atomic<double> _new_altitude_amsl;
    std::atomic<double> _new_x;
    std::atomic<double> _new_y;
    std::atomic<double> _new_z;
    float _new_yaw;
    std::atomic<double> _previously_set_waypoint_latitude;
    std::atomic<double> _previously_set_waypoint_longitude;
    std::atomic<double> _previously_set_waypoint_altitude_amsl;
    std::atomic<double> _original_max_speed;

    landing_planner::LandingPlanner _landing_planner;

    std::chrono::time_point<std::chrono::system_clock> _last_time{};

    std::thread _decision_maker_th;
    std::thread _global_origin_reference_th;

    bool _was_mission_paused;
    bool _was_landing_paused;

    double _landing_latitude_deg;
    double _landing_longitude_deg;
    double _landing_altitude_m;
    int _landing_waypoint_id;

    rclcpp::Time _time_last_traj;

    bool _got_traj;

    timing_tools::FrequencyMeter _frequency_traj;

    rclcpp::TimerBase::SharedPtr _timer_send_corrected_trajectory;
    mavlink_message_t _corrected_traj_message;
};
