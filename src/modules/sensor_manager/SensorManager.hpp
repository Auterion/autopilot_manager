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
 * @brief Sensor Manager
 * @file SensorManager.hpp
 * @author Nuno Marques <nuno@auterion.com>
 * @author Julian Kent <julian@auterion.com>
 */

#pragma once

#include <common.h>
#include <timing_tools/timing_tools.h>

#include <Eigen/Dense>
#include <ModuleBase.hpp>
#include <ObstacleAvoidanceModule.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>

#include "TimeSync.hpp"

// ROS dependencies
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// MAVSDK dependencies
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/server_utility/server_utility.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

inline static constexpr auto sensorManagerOut = "[Sensor Manager] ";

class SensorManager : public rclcpp::Node, public ObstacleAvoidanceModule, ModuleBase {
   public:
    SensorManager(std::shared_ptr<mavsdk::System> mavsdk_system);
    ~SensorManager();
    SensorManager(const SensorManager&) = delete;
    auto operator=(const SensorManager&) -> const SensorManager& = delete;

    auto init() -> void override;
    auto deinit() -> void override;
    auto run() -> void override;

    std::shared_ptr<ExtendedDownsampledImageF> RCPPUTILS_TSA_GUARDED_BY(_sensor_manager_mutex)
        get_lastest_downsampled_depth() {
        std::lock_guard<std::mutex> lock(_sensor_manager_mutex);
        return _downsampled_depth;
    }

    void set_camera_static_tf(const double x, const double y, const double yaw_deg);

   private:
    void handle_incoming_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void handle_incoming_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    bool set_downsampler(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    void health_check();

    mutable std::mutex _sensor_manager_mutex;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _depth_img_camera_info_sub;

    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_pub;  // for bagger in MAVLink mode

    std::shared_ptr<mavsdk::System> _mavsdk_system;
    std::shared_ptr<mavsdk::Telemetry> _telemetry;
    std::shared_ptr<mavsdk::ServerUtility> _server_utility;
    std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;

    std::shared_ptr<ImageDownsamplerInterface> _imageDownsampler;

    RectifiedIntrinsicsF _intrinsics;
    Eigen::Vector2f _inverse_focal_length;
    Eigen::Vector2f _principal_point;

    int16_t _downsampling_block_size;
    static constexpr float _downsampling_min_depth_to_use_m{0.2};

    tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::MessageFilter<sensor_msgs::msg::Image> _tf_depth_filter;

    message_filters::Subscriber<sensor_msgs::msg::Image> _tf_depth_subscriber;

    geometry_msgs::msg::TransformStamped _camera_static_tf;

    rclcpp::TimerBase::SharedPtr _timer_status_task;

    rclcpp::Time _time_last_odometry;
    rclcpp::Time _time_last_image;

    TimeSync _time_sync;

    timing_tools::FrequencyMeter _frequency_images;
    timing_tools::FrequencyMeter _frequency_camera_info;
    timing_tools::FrequencyMeter _frequency_odometry;

   protected:
    std::shared_ptr<ExtendedDownsampledImageF> _downsampled_depth;
};
