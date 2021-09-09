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
 * @file SensorManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <SensorManager.hpp>
#include <sensor_msgs/image_encodings.hpp>

SensorManager::SensorManager() : Node("sensor_manager"), _downsampline_block_size(8) {}

SensorManager::~SensorManager() { deinit(); }

void SensorManager::init() {
    std::cout << sensorManagerOut << " Started!" << std::endl;

    bool sim;
    this->declare_parameter("sim");
    this->get_parameter_or("sim", sim, false);

    rclcpp::SensorDataQoS qos;
    qos.keep_last(10);
    qos.best_effort();

    // Camera topic name changes for sim
    std::string depth_topic{"/camera/depth/image_rect_raw"};
    std::string depth_camera_info_topic{"/camera/depth/camera_info"};
    if (sim) {
        depth_topic = "/camera/depth/image_raw";
    }

    _depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { handle_incoming_depth_image(msg); });
    _depth_img_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_camera_info_topic, qos,
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { handle_incoming_camera_info(msg); });
}

auto SensorManager::deinit() -> void { _depth_image_sub.reset(); }

auto SensorManager::run() -> void { rclcpp::spin(shared_from_this()); }

void SensorManager::handle_incoming_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    const RectifiedIntrinsicsF raw_intrinsics(msg->k[0], msg->k[4], msg->k[2], msg->k[5], msg->width, msg->height);

    if (_imageDownsampler == nullptr) {
        return;
    }

    _imageDownsampler->adaptIntrinsics(raw_intrinsics, _intrinsics);

    _inverse_focal_length = _intrinsics.inverse_focal_length();
    _principal_point = _intrinsics.principal_point();
}

bool SensorManager::set_downsampler(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    bool ret = true;
    if (_imageDownsampler == nullptr) {
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            _imageDownsampler = ImageDownsamplerInterface::getInstance<uint16_t>(
                msg->width, msg->height, _downsampline_block_size, _downsampline_block_size, 0.2);

        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            _imageDownsampler = ImageDownsamplerInterface::getInstance<float>(
                msg->width, msg->height, _downsampline_block_size, _downsampline_block_size, 0.2);
        } else {
            RCLCPP_ERROR(get_logger(), "Unhandled image encoding %s", msg->encoding.c_str());
            ret = false;
        }
    }

    return ret;
}

void SensorManager::handle_incoming_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    set_downsampler(msg);

    if (_imageDownsampler == nullptr) {
        RCLCPP_ERROR_SKIPFIRST(get_logger(), "_imageDownsampler not set");
        return;
    }

    const bool intrinsicPlausible = (_intrinsics.rh != 0) && (_intrinsics.rw != 0);
    if (!intrinsicPlausible) {
        RCLCPP_ERROR_SKIPFIRST(get_logger(), "Intrinsics not plausible");
        return;
    }

    std::shared_ptr<DownsampledImageF> downsampled_depth_image = std::make_shared<DownsampledImageF>();

    downsampled_depth_image->depth_pixel_array = _imageDownsampler->downsample(msg->data.data());
    downsampled_depth_image->intrinsics = _intrinsics;

    // Make the downsampled depth data available for other modules
    std::lock_guard<std::mutex> lock(_sensor_manager_mutex);
    _downsampled_depth = downsampled_depth_image;
}
