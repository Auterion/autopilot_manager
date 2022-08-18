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

using namespace std::chrono_literals;

SensorManager::SensorManager(std::shared_ptr<mavsdk::System> mavsdk_system)
    : Node("sensor_manager"),
      _mavsdk_system{std::move(mavsdk_system)},
      _downsampling_block_size(4),
      _static_tf_broadcaster(this),
      _tf_broadcaster(this),
      _tf_buffer(this->get_clock()),
      _tf_listener(_tf_buffer),
      _tf_depth_filter(_tf_buffer, NED_FRAME, 10, this->create_sub_node("tf_filter")),
      _time_last_odometry{this->now()},
      _time_last_image{this->now()},
      _frequency_images("sensor images"),
      _frequency_camera_info("sensor camera_info"),
      _frequency_odometry("sensor odometry") {}

SensorManager::~SensorManager() { deinit(); }

void SensorManager::init() {
    std::cout << sensorManagerOut << "Started!" << std::endl;
    std::cout << sensorManagerOut << "Downsampling block size = " << _downsampling_block_size << std::endl;

    bool sim;
    this->declare_parameter("sim");
    this->get_parameter_or("sim", sim, false);

    rclcpp::SensorDataQoS qos;
    qos.keep_last(10);
    qos.best_effort();
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    // Camera topic name changes for sim
    std::string depth_topic{"/camera/depth/image_rect_raw"};
    std::string depth_camera_info_topic{"/camera/depth/camera_info"};
    if (sim) {
        depth_topic = "/camera/depth/image_raw";
    }

    _telemetry = std::make_shared<mavsdk::Telemetry>(_mavsdk_system);
    _server_utility = std::make_shared<mavsdk::ServerUtility>(_mavsdk_system);

    _mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(_mavsdk_system);

    _depth_img_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_camera_info_topic, qos,
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { handle_incoming_camera_info(msg); });

    _tf_depth_subscriber.subscribe(this, depth_topic, rmw_qos_profile);
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    _tf_buffer.setCreateTimerInterface(timer_interface);
    _tf_depth_filter.connectInput(_tf_depth_subscriber);
    _tf_depth_filter.registerCallback(&SensorManager::handle_incoming_depth_image, this);
    _tf_depth_filter.setTolerance(rclcpp::Duration(0, static_cast<int>(10 * 1E6)));

    _vehicle_status_pub =
        this->create_publisher<px4_msgs::msg::VehicleStatus>("vehicle_status/out", 10);  // for bagger in MAVLink mode
}

auto SensorManager::deinit() -> void { _depth_img_camera_info_sub.reset(); }

auto SensorManager::run() -> void {
    // Subscribe to odometry for publishing the TF
    _telemetry->subscribe_odometry([this](mavsdk::Telemetry::Odometry odometry) {
        _frequency_odometry.tic();

        geometry_msgs::msg::TransformStamped tMsg{};

        tMsg.transform.translation = geometry_msgs::msg::Vector3{};
        tMsg.transform.translation.x = odometry.position_body.x_m;
        tMsg.transform.translation.y = odometry.position_body.y_m;
        tMsg.transform.translation.z = odometry.position_body.z_m;

        tMsg.transform.rotation = geometry_msgs::msg::Quaternion{};
        tMsg.transform.rotation.w = odometry.q.w;
        tMsg.transform.rotation.x = odometry.q.x;
        tMsg.transform.rotation.y = odometry.q.y;
        tMsg.transform.rotation.z = odometry.q.z;

        uint64_t ts_ns = _time_sync.sync_stamp(odometry.time_usec, this->now().nanoseconds() / 1000ULL) * 1000;
        tMsg.header.stamp = rclcpp::Time(ts_ns);

        tMsg.header.frame_id = NED_FRAME;
        tMsg.child_frame_id = BASE_LINK_FRAME;

        _tf_broadcaster.sendTransform(tMsg);

        _time_last_odometry = this->now();
    });

    _telemetry->subscribe_armed([this](bool _armed) {
        px4_msgs::msg::VehicleStatus msg;
        if (_armed) {
            msg.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        } else {
            msg.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY;
        }

        _vehicle_status_pub->publish(msg);  // Send data for bagger
    });

    _mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_TIMESYNC, [this](const mavlink_message_t& _message) {
        mavlink_timesync_t tsync;
        mavlink_msg_timesync_decode(&_message, &tsync);

        _time_sync.run(tsync.ts1, tsync.tc1, this->now().nanoseconds() / 1000ULL);
    });

    _timer_status_task = create_wall_timer(100ms, std::bind(&SensorManager::health_check, this));

    rclcpp::spin(shared_from_this());
}

bool SensorManager::set_downsampler(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    bool ret = true;
    if (_imageDownsampler == nullptr) {
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            _imageDownsampler = ImageDownsamplerInterface::getInstance<uint16_t>(
                msg->width, msg->height, _downsampling_block_size, _downsampling_block_size,
                _downsampling_min_depth_to_use_m);

        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            _imageDownsampler = ImageDownsamplerInterface::getInstance<float>(
                msg->width, msg->height, _downsampling_block_size, _downsampling_block_size,
                _downsampling_min_depth_to_use_m);
        } else {
            RCLCPP_ERROR(get_logger(), "Unhandled image encoding %s", msg->encoding.c_str());
            ret = false;
        }
    }

    return ret;
}

void SensorManager::set_camera_static_tf(const double x, const double y, const double yaw_deg) {
    _camera_static_tf = geometry_msgs::msg::TransformStamped{};

    _camera_static_tf.transform.translation = geometry_msgs::msg::Vector3{};
    _camera_static_tf.transform.translation.x = x;
    _camera_static_tf.transform.translation.y = y;
    _camera_static_tf.transform.translation.z = 0.;

    tf2::Quaternion rot;
    const double yaw_rad = yaw_deg * M_PI / 180.;
    rot.setRPY(0., 0., yaw_rad + M_PI_2);
    rot = rot.normalize();
    _camera_static_tf.transform.rotation = geometry_msgs::msg::Quaternion{};
    _camera_static_tf.transform.rotation.w = rot.w();
    _camera_static_tf.transform.rotation.x = rot.x();
    _camera_static_tf.transform.rotation.y = rot.y();
    _camera_static_tf.transform.rotation.z = rot.z();

    _camera_static_tf.header.stamp = this->now();
    _camera_static_tf.header.frame_id = BASE_LINK_FRAME;
    _camera_static_tf.child_frame_id = CAMERA_LINK_FRAME;

    _static_tf_broadcaster.sendTransform(_camera_static_tf);

    std::cout << sensorManagerOut << "Camera offset is [" << x << "m, " << y << "m] with " << yaw_deg << "° yaw."
              << std::endl;
}

void SensorManager::handle_incoming_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    _frequency_camera_info.tic();

    const RectifiedIntrinsicsF raw_intrinsics(msg->k[0], msg->k[4], msg->k[2], msg->k[5], msg->width, msg->height);

    if (_imageDownsampler == nullptr) {
        return;
    }

    _imageDownsampler->adaptIntrinsics(raw_intrinsics, _intrinsics);

    _inverse_focal_length = _intrinsics.inverse_focal_length();
    _principal_point = _intrinsics.principal_point();
}

void SensorManager::handle_incoming_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    _frequency_images.tic();

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

    std::shared_ptr<ExtendedDownsampledImageF> downsampled_depth_image = std::make_shared<ExtendedDownsampledImageF>();

    downsampled_depth_image->downsampled_image.depth_pixel_array = _imageDownsampler->downsample(msg->data.data());
    downsampled_depth_image->downsampled_image.intrinsics = _intrinsics;

    // Get position and orientation to image
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = _tf_buffer.lookupTransform(NED_FRAME, CAMERA_LINK_FRAME, msg->header.stamp);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
        downsampled_depth_image = nullptr;
        return;
    }

    downsampled_depth_image->position =
        Eigen::Vector3f(transformStamped.transform.translation.x, transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z);
    downsampled_depth_image->orientation =
        Eigen::Quaternionf(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                           transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
    downsampled_depth_image->timestamp_ns = msg->header.stamp.nanosec;

    // Make the downsampled depth data available for other modules
    std::lock_guard<std::mutex> lock(_sensor_manager_mutex);
    _downsampled_depth = downsampled_depth_image;

    _time_last_image = this->now();
}

void SensorManager::health_check() {
    const auto now = this->now();
    static rclcpp::Time time_start = now;

    const auto s_since_start = (now - time_start).seconds();
    if (s_since_start < 5.0) {
        return;
    }

    const auto s_since_last_odom = (now - _time_last_odometry).seconds();
    const auto s_since_last_image = (now - _time_last_image).seconds();

    const bool is_odom_healthy = s_since_last_odom < std::chrono::duration<double>(2.5s).count();
    const bool is_image_healthy = s_since_last_image < std::chrono::duration<double>(2.5s).count();
    const bool is_healthy = is_odom_healthy && is_image_healthy;

    const rclcpp::Duration warning_interval(2, 0);
    static rclcpp::Time last_warning = now;
    const bool is_exceeded = this->now() > (last_warning + warning_interval);

    static bool health_reported_once = false;

    if (!is_healthy && is_exceeded) {
        last_warning = now;

        std::stringstream ss;
        ss << "Input unhealthy.";
        if (!is_odom_healthy) {
            ss << " - Odometry unhealthy.";
        }
        if (!is_image_healthy) {
            ss << " - Images unhealthy.";
        }

        RCLCPP_ERROR(get_logger(), ss.str());
        if (_server_utility) {
            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Alert, ss.str());
        }
    } else if (!health_reported_once) {
        health_reported_once = true;
        static const std::string good_to_go = "Safe Landing: Input healthy. Good to go!";
        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, good_to_go);
        std::cout << sensorManagerOut << good_to_go << std::endl;
    }

    mavlink_timesync_t timesync_message;
    timesync_message.ts1 = this->now().nanoseconds();

    mavlink_message_t message_out;
    mavlink_msg_timesync_encode(1, MAV_COMP_ID_ONBOARD_COMPUTER3, &message_out, &timesync_message);
    _mavlink_passthrough->send_message(message_out);
}
