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
 * @brief Landing Manager
 * @file LandingManager.hpp
 * @author Nuno Marques <nuno@auterion.com>
 * @author Bastian Jäeger <bastian@auterion.com>
 */

#pragma once

#include <common.h>
#include <timing_tools/timing_tools.h>

#include <Eigen/Core>
#include <ModuleBase.hpp>
#include <chrono>
#include <iostream>

// ROS dependencies
#include <MapVisualizer.hpp>
#include <landing_mapper/LandingMapper.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

// MAVSDK dependencies
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/server_utility/server_utility.h>

struct VehicleState {
    bool valid{false};
    Eigen::Vector3f position{NAN, NAN, NAN}, velocity{NAN, NAN, NAN}, acceleration{NAN, NAN, NAN};
    Eigen::Quaternionf orientation{NAN, NAN, NAN, NAN};
    Eigen::Vector3f angular_velocity{NAN, NAN, NAN};
};

static constexpr auto landingManagerOut = "[Landing Manager] ";

class LandingManager : public rclcpp::Node, ModuleBase {
   public:
    LandingManager(std::shared_ptr<mavsdk::System> mavsdk_system);
    ~LandingManager();
    LandingManager(const LandingManager&) = delete;
    auto operator=(const LandingManager&) -> const LandingManager& = delete;

    auto init() -> void override;
    auto deinit() -> void override;
    auto run() -> void override;

    struct LandingManagerConfiguration {
        uint8_t autopilot_manager_enabled = 0U;
        uint8_t safe_landing_enabled = 0U;
        double safe_landing_area_square_size = 0.0;
        double safe_landing_distance_to_ground = 0.0;
    };

    landing_mapper::eLandingMapperState RCPPUTILS_TSA_GUARDED_BY(_landing_manager_mutex)
        get_latest_landing_condition_state() {
        std::lock_guard<std::mutex> lock(_landing_manager_mutex);
        return _state;
    }

    landing_mapper::eLandingMapperState RCPPUTILS_TSA_GUARDED_BY(_landing_manager_mutex)
        get_landing_condition_state_at_position(float x, float y) {
        std::lock_guard<std::mutex> lock_manager(_landing_manager_mutex);
        std::lock_guard<std::mutex> lock_map(_map_mutex);
        return _mapper->computeLandingStateAtPositionXY(x, y);
    }

    float RCPPUTILS_TSA_GUARDED_BY(_landing_manager_mutex) get_latest_height_above_obstacle() {
        std::lock_guard<std::mutex> lock(_landing_manager_mutex);
        return _height_above_obstacle;
    }

    void getDownsampledDepthDataCallback(std::function<std::shared_ptr<ExtendedDownsampledImageF>()> callback) {
        _downsampled_depth_update_callback = callback;
    }

    void setConfigUpdateCallback(std::function<LandingManagerConfiguration()> callback) {
        _config_update_callback = callback;
    }

    bool setSearchAltitude_m(double altitude_m);
    bool setSearchWindow_m(double window_size_m);

   private:
    void initParameters();
    void updateParameters();
    void mapper();
    bool healthCheck(const std::shared_ptr<ExtendedDownsampledImageF>& depth_msg) const;

    void visualizeResult(landing_mapper::eLandingMapperState state, const Eigen::Vector3f& position,
                         const rclcpp::Time& timestamp);
    void visualizeGroundPlane(const Eigen::Vector3f& normal, const Eigen::Vector3f& position,
                              const rclcpp::Time& timestamp);
    void visualizeMap();

    void printStats();

    std::shared_ptr<mavsdk::System> _mavsdk_system;
    std::shared_ptr<mavsdk::ServerUtility> _server_utility;

    std::function<LandingManagerConfiguration()> _config_update_callback;

    std::unique_ptr<landing_mapper::LandingMapper<float>> _mapper;
    landing_mapper::LandingMapperParameter _mapper_parameter;
    bool _visualize;

    LandingManagerConfiguration _landing_manager_config;

    std::mutex landing_manager_config_mtx;

    std::shared_ptr<viz::MapVisualizer> _visualizer;

    timing_tools::FrequencyMeter _frequency_mapper;
    timing_tools::FrequencyMeter _frequency_visualise_map;
    rclcpp::TimerBase::SharedPtr _timer_stats;

    rclcpp::TimerBase::SharedPtr _timer_mapper;
    rclcpp::TimerBase::SharedPtr _timer_map_visualizer;

    int _images_processed = 0;
    int _points_processed = 0;
    int _points_received = 0;

    rclcpp::CallbackGroup::SharedPtr _callback_group_mapper;
    rclcpp::CallbackGroup::SharedPtr _callback_group_telemetry;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _landing_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _height_above_obstacle_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ground_slope_angle_pub;

    std::function<std::shared_ptr<ExtendedDownsampledImageF>()> _downsampled_depth_update_callback;

    mutable std::mutex _landing_manager_mutex;
    mutable std::mutex _map_mutex;

    std::vector<Eigen::Vector3f> _pointcloud_for_mapper;

    landing_mapper::eLandingMapperState _state;
    float _height_above_obstacle;
};
