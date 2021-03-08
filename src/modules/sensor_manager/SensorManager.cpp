/****************************************************************************
 *
 * Copyright 2021 Auterion. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Sensor Manager
 * @file SensorManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 * @author Julian Kent <julian@auterion.com>
 */

#include <SensorManager.hpp>

using namespace builtin_interfaces::msg;
using namespace sensor_msgs::msg;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

using PIXEL = uint16_t;

SensorManager::SensorManager()
    : Node("sensor_manager")
{
    init();
}

SensorManager::~SensorManager() {
    deinit();
}

void SensorManager::init() {
	std::cout << "[Sensor Manager] Started!" << std::endl;

    _depth_image_sub =
        create_subscription<Image>("/camera/depth/image_raw", 10, std::bind(&SensorManager::handle_incoming_depth_image, this, _1));
}

void SensorManager::deinit() {
    _depth_image_sub.reset();
}

void SensorManager::run() {
    // not used!
}

void SensorManager::handle_incoming_depth_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // make an Eigen wrapper around the memory
    auto img = Eigen::Map<const Eigen::Matrix<PIXEL,-1,-1>>((const PIXEL*)(&msg->data[0]), msg->height, msg->width);

    // make a local copy of the ROI settings
    ROISettings local_settings;
    {
        std::lock_guard<std::mutex> lock(_sensor_manager_mutex);
        local_settings = _roi_settings;
    }

    int64_t cols_pixels = static_cast<int64_t>(local_settings.width_fraction * img.cols());
    int64_t rows_pixels = static_cast<int64_t>(local_settings.height_fraction * img.rows());
    int64_t cols_offset = static_cast<int64_t>((local_settings.width_center - 0.5f * local_settings.width_fraction) * img.cols());
    int64_t rows_offset = static_cast<int64_t>((local_settings.height_center - 0.5f * local_settings.height_fraction) * img.rows());

    float depth = NAN;

    // check that settings are OK
    if (cols_pixels > 0 && cols_pixels < img.cols() && rows_pixels > 0 && rows_pixels < img.rows() &&
        cols_offset >= 0 && cols_offset + cols_pixels < img.cols() && rows_offset >= 0 && rows_offset + rows_pixels < img.rows()){
        depth = img.block(rows_offset, cols_offset, rows_pixels, cols_pixels).mean();
    }

    std::lock_guard<std::mutex> lock(_sensor_manager_mutex);
    _depth = depth;
}
