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
 * @file LandingManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 * @author Bastian Jäeger <bastian@auterion.com>
 */

#include <LandingManager.hpp>

using namespace std::chrono_literals;

LandingManager::LandingManager() : Node("landing_manager") {}

LandingManager::~LandingManager() { deinit(); }

void LandingManager::init() {
    std::cout << landingManagerOut << " Started!" << std::endl;

    // Mapper runs at 10hz
    _timer = this->create_wall_timer(100ms, std::bind(&LandingManager::mapper, this));
}

auto LandingManager::deinit() -> void {}

auto LandingManager::run() -> void { rclcpp::spin(shared_from_this()); }

void LandingManager::mapper() {
    // Here we capture the downsampled depth data computed in the SensorManager
    auto depth_msg = _downsampled_depth_update_callback();

    if (depth_msg != nullptr) {
        // TODO: Add mapper code and set `can_land()` <- Bastian
    }
}

void LandingManager::can_land() {
    // This variable will provide the landing condition state, stating to the Mission
    // Manager if it can land or not. Ideally this should be computed from an histeresys
    // so to capture the state in a time window.
    bool can_land = false;

    // Provide to the Mission Manager access to the status of the landing procedure
    std::lock_guard<std::mutex> lock(_landing_manager_mutex);
    _can_land = can_land;
}
