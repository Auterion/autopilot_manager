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

#include <Eigen/Core>
#include <ModuleBase.hpp>
#include <chrono>
#include <iostream>

// ROS dependencies
#include <common.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <MapVisualizer.hpp>
#include <SensorManager.hpp>
#include <landing_mapper/mapper/LandingMapper.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

struct VehicleState {
    bool valid{false};
    Eigen::Vector3f position{NAN, NAN, NAN}, velocity{NAN, NAN, NAN}, acceleration{NAN, NAN, NAN};
    Eigen::Quaternionf orientation{NAN, NAN, NAN, NAN};
    Eigen::Vector3f angular_velocity{NAN, NAN, NAN};
};

static constexpr auto landingManagerOut = "[Landing Manager]";

class LandingManager : public rclcpp::Node, ModuleBase {
   public:
    LandingManager();
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
    void handleIncomingVehicleOdometry(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

    void visualizeResult(landing_mapper::eLandingMapperState state, const Eigen::Vector3f& position,
                         const rclcpp::Time& timestamp);
    void visualizeMap();

    std::function<LandingManagerConfiguration()> _config_update_callback;

    std::unique_ptr<landing_mapper::LandingMapper<float>> _mapper;
    landing_mapper::LandingMapperParameter _mapper_parameter;
    bool _visualize;

    LandingManagerConfiguration _landing_manager_config;

    std::mutex landing_manager_config_mtx;

    std::shared_ptr<viz::MapVisualizer> _visualizer;

    rclcpp::TimerBase::SharedPtr _timer_mapper;
    rclcpp::TimerBase::SharedPtr _timer_map_visualizer;

    rclcpp::CallbackGroup::SharedPtr _callback_group_mapper;
    rclcpp::CallbackGroup::SharedPtr _callback_group_telemetry;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _landing_state_pub;

    tf2_ros::TransformBroadcaster _tf_broadcaster;

    std::mutex _vehicle_state_mutex;
    std::unique_ptr<VehicleState> _vehicle_state;

    std::function<std::shared_ptr<ExtendedDownsampledImageF>()> _downsampled_depth_update_callback;

    mutable std::mutex _landing_manager_mutex;
    mutable std::mutex _map_mutex;

    std::vector<Eigen::Vector3f> _pointcloud_for_mapper;

    landing_mapper::eLandingMapperState _state;
};
