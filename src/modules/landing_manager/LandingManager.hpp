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

#include <ModuleBase.hpp>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <iostream>

// ROS dependencies
#include <image_downsampler/DataTypes.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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

    bool RCPPUTILS_TSA_GUARDED_BY(_landing_manager_mutex) get_latest_landing_condition_state() {
        std::lock_guard<std::mutex> lock(_landing_manager_mutex);
        return _can_land;
    }

    void getDownsampledDepthDataCallback(std::function<std::shared_ptr<DownsampledImageF>()> callback) {
        _downsampled_depth_update_callback = callback;
    }

   private:
    void mapper();
    void can_land();

    rclcpp::TimerBase::SharedPtr _timer{};

    std::function<std::shared_ptr<DownsampledImageF>()> _downsampled_depth_update_callback;

    mutable std::mutex _landing_manager_mutex;

    bool _can_land{false};
};
