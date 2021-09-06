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
 * @brief Collision Avoidance Manager
 * @file CollisionAvoidance.hpp
 * @author Nuno Marques <nuno@auterion.com>
 * @author Julian Kent <julian@auterion.com>
 */

#pragma once

#include <ModuleBase.hpp>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <iostream>

// ROS dependencies
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

static constexpr auto collisionAvoidanceManagerOut = "[Collision Avoidance Manager]";

class CollisionAvoidanceManager : public rclcpp::Node, ModuleBase {
   public:
    CollisionAvoidanceManager();
    ~CollisionAvoidanceManager();
    CollisionAvoidanceManager(const CollisionAvoidanceManager&) = delete;
    auto operator=(const CollisionAvoidanceManager&) -> const CollisionAvoidanceManager& = delete;

    auto init() -> void override;
    auto deinit() -> void override;
    auto run() -> void override;

    struct ROISettings {
        float width_fraction{0.2f};
        float height_fraction{0.2f};
        float width_center{0.5f};
        float height_center{0.5f};
    };

    void RCPPUTILS_TSA_GUARDED_BY(_collision_avoidance_manager_mutex) set_roi(const ROISettings& settings) {
        std::lock_guard<std::mutex> lock(_collision_avoidance_manager_mutex);
        _roi_settings = settings;
    }

    float RCPPUTILS_TSA_GUARDED_BY(_collision_avoidance_manager_mutex) get_latest_distance() {
        std::lock_guard<std::mutex> lock(_collision_avoidance_manager_mutex);
        return _depth;
    }

    void getDownsampledDepthDataCallback(std::function<std::shared_ptr<sensor_msgs::msg::Image>()> callback) {
        _downsampled_depth_update_callback = callback;
    }

   private:
    void compute_distance_to_obstacle();
    std::function<std::shared_ptr<sensor_msgs::msg::Image>()> _downsampled_depth_update_callback;

    rclcpp::TimerBase::SharedPtr _timer{};
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _obstacle_distance_pub{};

    mutable std::mutex _collision_avoidance_manager_mutex;

    ROISettings _roi_settings{};
    float _depth{NAN};
};
