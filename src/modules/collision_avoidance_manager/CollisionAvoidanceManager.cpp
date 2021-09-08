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
 * @file CollisionAvoidanceManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 * @author Julian Kent <julian@auterion.com>
 */

#include <CollisionAvoidanceManager.hpp>

using namespace std::chrono_literals;

CollisionAvoidanceManager::CollisionAvoidanceManager() : Node("collision_avoidance_manager") {}

CollisionAvoidanceManager::~CollisionAvoidanceManager() { deinit(); }

void CollisionAvoidanceManager::init() {
    std::cout << collisionAvoidanceManagerOut << " Started!" << std::endl;

    _obstacle_distance_pub =
        this->create_publisher<std_msgs::msg::Float32>("/collision_avoidance_manager/distance_to_obstacle", 10);

    // Distance to obstacle calculation runs at 10hz
    _timer = this->create_wall_timer(100ms, std::bind(&CollisionAvoidanceManager::compute_distance_to_obstacle, this));
}

auto CollisionAvoidanceManager::deinit() -> void { _obstacle_distance_pub.reset(); }

auto CollisionAvoidanceManager::run() -> void { rclcpp::spin(shared_from_this()); }

void CollisionAvoidanceManager::compute_distance_to_obstacle() {
    auto depth_msg = _downsampled_depth_update_callback();

    if (depth_msg != nullptr) {
        std::cout << collisionAvoidanceManagerOut << " IMAGE!" << std::endl;
        //        // Make an Eigen wrapper around the memory
        //        const auto img = Eigen::Map<const Eigen::Matrix<float, -1,
        //        -1>>(reinterpret_cast<float*>(&depth_msg->data[0]),
        //                                                                        depth_msg->height, depth_msg->width);
        //
        //        // Make a local copy of the ROI settings
        //        ROISettings local_settings;
        //        {
        //            std::lock_guard<std::mutex> lock(_collision_avoidance_manager_mutex);
        //            local_settings = _roi_settings;
        //        }
        //
        //        const int64_t cols_pixels = static_cast<int64_t>(local_settings.width_fraction * img.cols());
        //        const int64_t rows_pixels = static_cast<int64_t>(local_settings.height_fraction * img.rows());
        //        const int64_t cols_offset =
        //            static_cast<int64_t>((local_settings.width_center - 0.5f * local_settings.width_fraction) *
        //            img.cols());
        //        const int64_t rows_offset =
        //            static_cast<int64_t>((local_settings.height_center - 0.5f * local_settings.height_fraction) *
        //            img.rows());
        //
        //        float depth = std::numeric_limits<float>::infinity();
        //
        //        // Check that settings are OK
        //        if (cols_pixels > 0 && cols_pixels < img.cols() && rows_pixels > 0 && rows_pixels < img.rows() &&
        //            cols_offset >= 0 && cols_offset + cols_pixels < img.cols() && rows_offset >= 0 &&
        //            rows_offset + rows_pixels < img.rows()) {
        //            depth = img.block(rows_offset, cols_offset, rows_pixels, cols_pixels)
        //                        .array()
        //                        .isNaN()
        //                        .select(std::numeric_limits<float>::infinity(),
        //                                img.block(rows_offset, cols_offset, rows_pixels, cols_pixels))
        //                        .minCoeff();
        //        }
        //
        //        // Make the obstacle distance available for the Mission Manager to access
        //        std::lock_guard<std::mutex> lock(_collision_avoidance_manager_mutex);
        //        _depth = depth;
        //
        //        // Publish obstacle distance back to ROS
        //        auto obstacle_dist = std_msgs::msg::Float32();
        //        obstacle_dist.data = _depth;
        //        _obstacle_distance_pub->publish(obstacle_dist);
    }
}
