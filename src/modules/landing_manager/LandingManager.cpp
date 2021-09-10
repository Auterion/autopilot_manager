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
using namespace std::placeholders;

LandingManager::LandingManager()
    : Node("landing_manager"),
      _visualize(true),
      _visualizer(std::make_shared<viz::MapVisualizer>(this)),
      _timer_mapper({}),
      _timer_map_visualizer({}),
      _tf_broadcaster(this),
      _tf_buffer(this->get_clock()),
      _tf_listener(_tf_buffer),
      _vehicle_state(new VehicleState()),
      _state(landing_mapper::eLandingMapperState::UNKNOWN) {}

LandingManager::~LandingManager() { deinit(); }

void LandingManager::initParameter() {
    _mapper_parameter.max_search_altitude_m = 22.f;
    _mapper_parameter.search_altitude_m = 10.f;  // TODO ensure this is smaller max_search_altitude_m
    _mapper_parameter.max_window_size_m = 5.f;
    _mapper_parameter.window_size_m = 2.f;  // TODO ensure this is smaller max_window_size_m

    _mapper_parameter.distance_threshold_m = 0.1f;
    _mapper_parameter.mean_tresh = 0.1f;
    _mapper_parameter.percentage_of_valid_samples_in_window = 0.8f;
    _mapper_parameter.std_dev_tresh = 0.05f;
    _mapper_parameter.voxel_size_m = 0.1f;
}
void LandingManager::init() {
    std::cout << landingManagerOut << " Started!" << std::endl;

    initParameter();

    _mapper = std::make_unique<landing_mapper::LandingMapper<float>>(_mapper_parameter);

    // Setup ROS stuff
    _callback_group_mapper = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto mapper_opt = rclcpp::SubscriptionOptions();
    mapper_opt.callback_group = _callback_group_mapper;
    _callback_group_telemetry = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto telemetry_opt = rclcpp::SubscriptionOptions();
    telemetry_opt.callback_group = _callback_group_telemetry;

    // Mapper runs at 10hz
    _timer_mapper = this->create_wall_timer(100ms, std::bind(&LandingManager::mapper, this), _callback_group_mapper);

    // Vizualizaion runs at 1hz
    _timer_map_visualizer = this->create_wall_timer(1000ms, std::bind(&LandingManager::visualizeMap, this));

    // Setup odometry subscriber
    _vehicle_odometry_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/vehicle_odometry/out", 10, std::bind(&LandingManager::handleIncomingVehicleOdometry, this, _1),
        telemetry_opt);
}

auto LandingManager::deinit() -> void {}

auto LandingManager::run() -> void { rclcpp::spin(shared_from_this()); }

bool LandingManager::setSearchAltitude_m(float altitude) {
    if (altitude < 0.0) {
        return false;
    } else if (altitude > _mapper_parameter.max_search_altitude_m) {
        return false;
    } else {
        _mapper_parameter.max_search_altitude_m = altitude;
    }

    return true;
}

bool LandingManager::setSearchWindow_m(float window_size_m) {
    if (window_size_m < 0.0) {
        return false;
    } else if (window_size_m > _mapper_parameter.window_size_m) {
        return false;
    } else {
        _mapper_parameter.window_size_m = window_size_m;
    }

    return true;
}

void LandingManager::handleIncomingVehicleOdometry(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
    {
        std::lock_guard<std::mutex> lock(_vehicle_state_mutex);

        _vehicle_state->position = Eigen::Vector3f(msg->x, msg->y, msg->z);
        _vehicle_state->orientation = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

        _vehicle_state->velocity = Eigen::Vector3f(msg->vx, msg->vy, msg->vz);
        _vehicle_state->angular_velocity = Eigen::Vector3f(msg->rollspeed, msg->pitchspeed, msg->yawspeed);

        _vehicle_state->acceleration = Eigen::Vector3f(msg->ax, msg->ay, msg->az);

        _vehicle_state->valid = true;
    }

    geometry_msgs::msg::TransformStamped tMsg{};

    tMsg.transform.translation = geometry_msgs::msg::Vector3{};
    tMsg.transform.translation.x = _vehicle_state->position.x();
    tMsg.transform.translation.y = _vehicle_state->position.y();
    tMsg.transform.translation.z = _vehicle_state->position.z();

    tMsg.transform.rotation = geometry_msgs::msg::Quaternion{};
    tMsg.transform.rotation.w = _vehicle_state->orientation.w();
    tMsg.transform.rotation.x = _vehicle_state->orientation.x();
    tMsg.transform.rotation.y = _vehicle_state->orientation.y();
    tMsg.transform.rotation.z = _vehicle_state->orientation.z();

    tMsg.header.stamp.sec = msg->timestamp / 1000000;
    tMsg.header.stamp.nanosec = (msg->timestamp - tMsg.header.stamp.sec * 1000000) * 1000;
    tMsg.header.frame_id = NED_FRAME;
    tMsg.child_frame_id = BASE_LINK_FRAME;

    _tf_broadcaster.sendTransform(tMsg);
}

std::string string_state(landing_mapper::eLandingMapperState state) {
    switch (state) {
        case landing_mapper::eLandingMapperState::CAN_LAND:
            return "CAN_LAND";
            break;
        case landing_mapper::eLandingMapperState::CAN_NOT_LAND:
            return "CAN_NOT_LAND";
            break;
        case landing_mapper::eLandingMapperState::CLOSE_TO_GROUND:
            return "CLOSE_TO_GROUND";
            break;
        default:
            return "UNKNOWN";
    }
}

void LandingManager::mapper() {
    // Here we capture the downsampled depth data computed in the SensorManager
    std::shared_ptr<DownsampledImageF> depth_msg = _downsampled_depth_update_callback();

    if (depth_msg != nullptr && depth_msg->depth_pixel_array.size() > 0) {
        const rclcpp::Time timenow = now();

        // Input new image data
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = _tf_buffer.lookupTransform(NED_FRAME, CAMERA_LINK_FRAME, rclcpp::Time(0));
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(get_logger(), "%s", ex.what());
            return;
        }
        const Eigen::Vector3f trans(transformStamped.transform.translation.x, transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.z);
        const Eigen::Quaternionf quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                                      transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
        {
            std::lock_guard<std::mutex> lock(_map_mutex);
            _pointcloud_for_mapper.clear();
            _visualizer->prepare_point_cloud_msg(timenow.nanoseconds(), depth_msg->intrinsics.rw,
                                                 depth_msg->intrinsics.rh, _visualize);

            const Eigen::Vector2f principal_point = depth_msg->intrinsics.principal_point();
            const Eigen::Vector2f inverse_focal_length = depth_msg->intrinsics.inverse_focal_length();

            for (const DepthPixelF& depth_pixel : depth_msg->depth_pixel_array) {
                const float depth = depth_pixel.depth;

                if (std::isfinite(depth) && (depth > 0.3f) &&
                    (depth < 20.f)) {  // TODO make 0.3 and 20.f parameter again
                    Eigen::Matrix<float, 3, 1> point(0.0, 0.0, depth);
                    point.head<2>() = (Eigen::Matrix<float, 2, 1>(depth_pixel.x, depth_pixel.y) - principal_point)
                                          .cwiseProduct(inverse_focal_length) *
                                      depth;
                    point = quat * point + trans;

                    _pointcloud_for_mapper.push_back(point);
                    _visualizer->add_point_to_point_cloud(point, _visualize);
                }
            }

            _mapper->updateCloud(_pointcloud_for_mapper);

            _visualizer->visualizePointCloud(_visualize);
        }

        // Find plain ground
        VehicleState local_state;
        {
            std::lock_guard<std::mutex> lock(_vehicle_state_mutex);
            local_state = *_vehicle_state;
        }
        _mapper->updateVehiclePosition(local_state.position, local_state.velocity, local_state.acceleration);
        _mapper->updateVehicleOrientation(local_state.orientation, local_state.angular_velocity.z(), 0);

        Eigen::Vector3f ground_position;

        landing_mapper::eLandingMapperState state = _mapper->findPlain(ground_position);
        state = stateDebounce(state);

        {
            std::lock_guard<std::mutex> lock(_landing_manager_mutex);
            _state = state;
        }

        // Show result
        visualizeResult(state, ground_position, timenow);
        // std::cout << landingManagerOut << " height " << ground_position.z() - local_state.position.z() << " state "
        // << string_state(state) << std::endl;
    } else {
        std::lock_guard<std::mutex> lock(_landing_manager_mutex);
        if (_state != landing_mapper::eLandingMapperState::CLOSE_TO_GROUND) {
            _state = landing_mapper::eLandingMapperState::UNKNOWN;
        }

        // Show result
        // std::cout << landingManagerOut << " state " << string_state(_state) << std::endl;
    }
}

landing_mapper::eLandingMapperState LandingManager::stateDebounce(landing_mapper::eLandingMapperState state) {
    landing_mapper::eLandingMapperState old_state;
    {
        std::lock_guard<std::mutex> lock(_landing_manager_mutex);
        old_state = _state;
    }

    constexpr float hysteresis_low_thresh = 0.7;
    constexpr float hysteresis_high_thresh = 0.8;
    constexpr size_t hysteresis_window_size = 10;

    _states.push_back(state);
    while (_states.size() > hysteresis_window_size) {
        _states.pop_front();
    }
    float avg = static_cast<float>(
                    std::count(_states.cbegin(), _states.cend(), landing_mapper::eLandingMapperState::CAN_LAND)) /
                _states.size();

    if (state == landing_mapper::eLandingMapperState::CAN_LAND) {
        if (avg < hysteresis_high_thresh) {
            state = old_state;
        } else {
            state = landing_mapper::eLandingMapperState::CAN_LAND;
        }
    } else {
        if (avg > hysteresis_low_thresh) {
            state = landing_mapper::eLandingMapperState::CAN_LAND;
        } else {
            state = state;
        }
    }

    {
        std::lock_guard<std::mutex> lock(_landing_manager_mutex);
        _state = state;
    }

    return state;
}

void LandingManager::visualizeResult(landing_mapper::eLandingMapperState state, const Eigen::Vector3f& position,
                                     const rclcpp::Time& timestamp) {
    Eigen::Vector3f vis_position(position[0], position[1], position[2] - 0.5);
    if (state == landing_mapper::eLandingMapperState::CAN_LAND) {
        _visualizer->publishSafeLand(vis_position, timestamp, _mapper_parameter.window_size_m, _visualize);
    } else if (state == landing_mapper::eLandingMapperState::CLOSE_TO_GROUND) {
        _visualizer->publishCloseGround(vis_position, timestamp, _mapper_parameter.window_size_m, _visualize);
    } else {
        _visualizer->publishGround(vis_position, timestamp, _mapper_parameter.window_size_m, _visualize);
    }
}

void LandingManager::visualizeMap() { _visualizer->visualizeEsdf(_mapper->getEsdf(), now(), _visualize); }
