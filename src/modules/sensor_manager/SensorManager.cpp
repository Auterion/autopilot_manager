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

SensorManager::SensorManager() : Node("sensor_manager") {}

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
    if (sim) {
        depth_topic = "/camera/depth/image_raw";
    }

    _depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { handle_incoming_depth_image(msg); });
}

auto SensorManager::deinit() -> void { _depth_image_sub.reset(); }

auto SensorManager::run() -> void { rclcpp::spin(shared_from_this()); }

void SensorManager::handle_incoming_depth_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Allocate new Image message
    auto downsampled_depth = std::make_shared<sensor_msgs::msg::Image>();
    downsampled_depth->header = msg->header;
    downsampled_depth->height = msg->height;
    downsampled_depth->width = msg->width;

    // If the pixels are encoded in uint16_t, we encode it in floats and convert
    // from millimeters to meters
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        // Set data, encoding and step after converting the metric.
        downsampled_depth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        downsampled_depth->step = msg->width * (sensor_msgs::image_encodings::bitDepth(downsampled_depth->encoding) / 8);
        downsampled_depth->data.resize(downsampled_depth->height * downsampled_depth->step);
        // Fill in the depth image data, converting mm to m
        const float bad_point = std::numeric_limits<float>::quiet_NaN();
        const uint16_t* raw_data = reinterpret_cast<const uint16_t*>(&msg->data[0]);
        float* depth_data = reinterpret_cast<float*>(&downsampled_depth->data[0]);
        for (unsigned index = 0; index < downsampled_depth->height * downsampled_depth->width; ++index) {
            uint16_t raw = raw_data[index];
            depth_data[index] = (raw == 0) ? bad_point : static_cast<float>(raw) * 0.001f;
        }
    } else {
        downsampled_depth->data = msg->data;
    }

    // TODO: add downsampling -> Bastian

    // Make the downsampled depth data available for other modules
    std::lock_guard<std::mutex> lock(_sensor_manager_mutex);
    _downsampled_depth = downsampled_depth;
}
