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

CollisionAvoidanceManager::CollisionAvoidanceManager()
    : Node("collision_avoidance_manager"),
      _config_update_callback([]() { return CollisionAvoidanceManagerConfiguration{}; }) {}

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

bool CollisionAvoidanceManager::is_pixel_valid(const DepthPixelF& pixel, uint32_t col_min, uint32_t col_max,
                                               uint32_t row_min, uint32_t row_max) const {
    if (pixel.x < col_min || pixel.x > col_max) {
        return false;
    }
    if (pixel.y < row_min || pixel.y > row_max) {
        return false;
    }

    return true;
}

void CollisionAvoidanceManager::filter_pixels_to_roi(DepthPixelArrayF& depth_pixel_array,
                                                     const RectifiedIntrinsicsF& intrinsics) {
    ROISettings roi;
    {
        std::lock_guard<std::mutex> lock(_collision_avoidance_manager_mutex);
        roi = _roi_settings;
    }

    const uint32_t col_min = static_cast<uint32_t>((roi.width_center - 0.5f * roi.width_fraction) * intrinsics.rw);
    const uint32_t col_max = static_cast<uint32_t>((roi.width_center + 0.5f * roi.width_fraction) * intrinsics.rw);
    const uint32_t row_min = static_cast<uint32_t>((roi.height_center - 0.5f * roi.height_fraction) * intrinsics.rh);
    const uint32_t row_max = static_cast<uint32_t>((roi.height_center + 0.5f * roi.height_fraction) * intrinsics.rh);

    for (auto pixel = depth_pixel_array.begin(); pixel != depth_pixel_array.end();) {
        if (!is_pixel_valid(*pixel, col_min, col_max, row_min, row_max)) {
            pixel = depth_pixel_array.erase(pixel);
        } else {
            ++pixel;
        }
    }
}

void CollisionAvoidanceManager::compute_distance_to_obstacle() {
    // update parameters
    // TODO: make this call dependent on a dbus param update on the Autopilot Manager
    // instead of running at every loop update
    _collision_avoidance_manager_config = _config_update_callback();

    auto depth_msg = _downsampled_depth_update_callback();

    if (depth_msg != nullptr && depth_msg->downsampled_image.depth_pixel_array.size() > 0) {
        // Get min depth in ROI
        DepthPixelArrayF depth_pixel_array = depth_msg->downsampled_image.depth_pixel_array;
        filter_pixels_to_roi(depth_pixel_array, depth_msg->downsampled_image.intrinsics);
        auto depth_pixel_compare = [](const DepthPixelF& lhs, const DepthPixelF& rhs) { return lhs.depth < rhs.depth; };
        const auto min_depth_pixel =
            std::min_element(depth_pixel_array.begin(), depth_pixel_array.end(), depth_pixel_compare);

        // Make the obstacle distance available for the Mission Manager to access
        {
            std::lock_guard<std::mutex> lock(_collision_avoidance_manager_mutex);
            _depth = min_depth_pixel->depth;
        }

        // Publish obstacle distance back to ROS
        auto obstacle_dist = std_msgs::msg::Float32();
        obstacle_dist.data = min_depth_pixel->depth;
        _obstacle_distance_pub->publish(obstacle_dist);
    } else {
        {
            std::lock_guard<std::mutex> lock(_collision_avoidance_manager_mutex);
            _depth = std::numeric_limits<float>::infinity();
        }
    }
}
