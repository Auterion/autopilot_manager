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

#include <Eigen/Dense>
#include <ModuleBase.hpp>
#include <chrono>
#include <iostream>

// ROS dependencies
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

inline static constexpr auto sensorManagerOut = "[Sensor  Manager]";

class SensorManager : public rclcpp::Node, ModuleBase {
   public:
    SensorManager();
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

   private:
    void handle_incoming_vehicle_odometry(const px4_msgs::msg::VehicleOdometry::ConstSharedPtr& msg);
    void handle_incoming_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void handle_incoming_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    bool set_downsampler(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    mutable std::mutex _sensor_manager_mutex;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _depth_img_camera_info_sub;

    std::shared_ptr<ImageDownsamplerInterface> _imageDownsampler;

    RectifiedIntrinsicsF _intrinsics;
    Eigen::Vector2f _inverse_focal_length;
    Eigen::Vector2f _principal_point;

    int16_t _downsampline_block_size;

    tf2_ros::TransformBroadcaster _tf_broadcaster;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::MessageFilter<sensor_msgs::msg::Image> _tf_depth_filter;

    message_filters::Subscriber<sensor_msgs::msg::Image> _tf_depth_subscriber;

   protected:
    std::shared_ptr<ExtendedDownsampledImageF> _downsampled_depth;
};
