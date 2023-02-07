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

static constexpr auto mapper_interval = 100ms;
static constexpr auto visualisation_interval = 1s;
static constexpr auto print_stats_interval = 5s;

LandingManager::LandingManager(std::shared_ptr<mavsdk::System> mavsdk_system)
    : Node("landing_manager"),
      _mavsdk_system{std::move(mavsdk_system)},
      _config_update_callback([]() { return LandingManagerConfiguration{}; }),
      _visualize(true),
      _visualizer(std::make_shared<viz::MapVisualizer>(this)),
      _frequency_mapper("mapper"),
      _frequency_visualise_map("visualise map"),
      _timer_stats(create_wall_timer(print_stats_interval, std::bind(&LandingManager::printStats, this))),
      _timer_mapper({}),
      _timer_map_visualizer({}),
      _health_status{HealthStatus::HEALTHY},
      _state(landing_mapper::eLandingMapperState::UNKNOWN),
      _height_above_obstacle{0.f} {
    _server_utility = std::make_shared<mavsdk::ServerUtility>(_mavsdk_system);
}

LandingManager::~LandingManager() { deinit(); }

void LandingManager::initParameters() {
    std::unique_lock<std::mutex> lock(landing_manager_config_mtx);
    _landing_manager_config = _config_update_callback();

    _mapper_parameter.search_altitude_m = 7.5f;
    _mapper_parameter.window_size_m = 2.0f;

    // Declare supported ROS parameters
    // Map config
    this->declare_parameter("max_search_altitude_m");
    this->declare_parameter("max_window_size_m");
    this->declare_parameter("voxel_size_m");
    // Safe-to-land parameters
    this->declare_parameter("slope_threshold_deg");
    this->declare_parameter("below_plane_deviation_thresh_m");
    this->declare_parameter("above_plane_deviation_thresh_m");
    this->declare_parameter("std_dev_from_plane_thresh_m");
    this->declare_parameter("percentage_of_valid_samples_in_window");
    // Enable debug logging
    this->declare_parameter("debug_mapper");

    // Get ROS parameters with defaults
    // Map config
    this->get_parameter_or("max_search_altitude_m", _mapper_parameter.max_search_altitude_m, 8);
    this->get_parameter_or("max_window_size_m", _mapper_parameter.max_window_size_m, 8);
    this->get_parameter_or("voxel_size_m", _mapper_parameter.voxel_size_m, 0.1f);
    // Safe-to-land parameters
    this->get_parameter_or("slope_threshold_deg", _mapper_parameter.slope_threshold_deg, 10.f);
    this->get_parameter_or("below_plane_deviation_thresh_m", _mapper_parameter.below_plane_deviation_thresh_m, 0.3f);
    this->get_parameter_or("above_plane_deviation_thresh_m", _mapper_parameter.above_plane_deviation_thresh_m, 0.3f);
    this->get_parameter_or("std_dev_from_plane_thresh_m", _mapper_parameter.std_dev_from_plane_thresh_m, 0.1f);
    this->get_parameter_or("percentage_of_valid_samples_in_window",
                           _mapper_parameter.percentage_of_valid_samples_in_window, 0.7f);
    // Enable debug logging
    this->get_parameter_or("debug_mapper", _mapper_parameter.debug_print, false);
}

void LandingManager::updateParameters() {
    std::unique_lock<std::mutex> lock(landing_manager_config_mtx);
    _landing_manager_config = _config_update_callback();

    // These are the only parameters configurable through AMC
    if (_landing_manager_config.safe_landing_distance_to_ground == 0 ||
        !setSearchAltitude_m(_landing_manager_config.safe_landing_distance_to_ground)) {
        _mapper_parameter.search_altitude_m = 7.5f;
    }
    if (_landing_manager_config.safe_landing_area_square_size == 0 ||
        !setSearchWindow_m(_landing_manager_config.safe_landing_area_square_size)) {
        _mapper_parameter.window_size_m = 2.0f;
    }
}

void LandingManager::init() {
    std::cout << landingManagerOut << "Started!" << std::endl;

    initParameters();
    updateParameters();

    _mapper = std::make_unique<landing_mapper::LandingMapper<float>>(_mapper_parameter);

    // Setup ROS stuff
    _callback_group_mapper = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto mapper_opt = rclcpp::SubscriptionOptions();
    mapper_opt.callback_group = _callback_group_mapper;
    _callback_group_telemetry = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto telemetry_opt = rclcpp::SubscriptionOptions();
    telemetry_opt.callback_group = _callback_group_telemetry;

    // Mapper runs at 10hz
    _timer_mapper =
        this->create_wall_timer(mapper_interval, std::bind(&LandingManager::mapper, this), _callback_group_mapper);

    // Vizualizaion runs at 1hz
    _timer_map_visualizer =
        this->create_wall_timer(visualisation_interval, std::bind(&LandingManager::visualizeMap, this));

    // Setup landing state publisher
    _landing_state_pub = this->create_publisher<std_msgs::msg::String>("landing_manager/landing_state", 10);

    // Setup height above obstacle publisher
    _height_above_obstacle_pub =
        this->create_publisher<std_msgs::msg::Float32>("landing_manager/height_above_obstacle", 10);

    // Setup publishers for the height map statistics
    _valid_sample_percentage_pub =
        this->create_publisher<std_msgs::msg::Float32>("landing_manager/stats/valid_sample_percentage", 10);
    _slope_angle_pub = this->create_publisher<std_msgs::msg::Float32>("landing_manager/stats/slope_angle", 10);
    _above_plane_max_deviation_pub =
        this->create_publisher<std_msgs::msg::Float32>("landing_manager/stats/above_plane_max_deviation", 10);
    _below_plane_max_deviation_pub =
        this->create_publisher<std_msgs::msg::Float32>("landing_manager/stats/below_plane_max_deviation", 10);
    _std_dev_from_plane_pub =
        this->create_publisher<std_msgs::msg::Float32>("landing_manager/stats/std_dev_from_plane", 10);
}

auto LandingManager::deinit() -> void {}

auto LandingManager::run() -> void { rclcpp::spin(shared_from_this()); }

bool LandingManager::setSearchAltitude_m(const double altitude) {
    if (altitude < 1.0) {
        return false;
    } else if (altitude > _mapper_parameter.max_search_altitude_m) {
        return false;
    } else {
        _mapper_parameter.search_altitude_m = altitude;
    }

    return true;
}

bool LandingManager::setSearchWindow_m(const double window_size) {
    if (window_size < 1.0) {
        return false;
    } else if (window_size > _mapper_parameter.max_window_size_m) {
        return false;
    } else {
        _mapper_parameter.window_size_m = window_size;
    }

    return true;
}

bool LandingManager::healthCheck(const std::shared_ptr<ExtendedDownsampledImageF>& depth_msg) {
    static constexpr int16_t MAX_NULL_IMAGE = 5;
    static constexpr int16_t MAX_OLD_TIMESTAMP = 50;

    struct HealthHandly {
        int16_t count_image_null{0};
        int16_t count_timestamp_old{0};

        int64_t last_timestamp{0};
    };
    static HealthHandly health;

    if (depth_msg == nullptr) {
        health.count_image_null++;
    } else {
        health.count_image_null = 0;

        const bool is_timestamp_old = health.last_timestamp >= depth_msg->timestamp_ns;
        health.last_timestamp = depth_msg->timestamp_ns;
        if (is_timestamp_old) {
            health.count_timestamp_old++;
        } else {
            health.count_timestamp_old = 0;
        }
    }

    const bool too_many_null_images = health.count_image_null > MAX_NULL_IMAGE;
    const bool too_many_old_timestamps = health.count_timestamp_old > MAX_OLD_TIMESTAMP;

    const HealthStatus new_health_status =
        static_cast<HealthStatus>((HealthStatus::UNHEALTHY_NULL_IMAGES * too_many_null_images) +
                                  (HealthStatus::UNHEALTHY_OLD_TIMESTAMPS * too_many_old_timestamps));
    const bool status_changed = new_health_status != _health_status;
    _health_status = new_health_status;

    if (!isHealthy() && status_changed) {
        std::stringstream ss;
        ss << "Mapper input unhealthy";
        if (too_many_null_images) {
            ss << " - NULL-images (" << health.count_image_null << ")";
        }
        if (too_many_old_timestamps) {
            ss << " - Old timestamps (" << health.count_timestamp_old << ")";
        }

        const std::string error_string = ss.str();
        RCLCPP_ERROR(get_logger(), error_string.c_str());
        if (_server_utility) {
            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Alert, error_string);
        }
    }

    return isHealthy();
}

void LandingManager::mapper() {
    _frequency_mapper.tic();

    // check for parameter updates
    // TODO: make this call dependent on a dbus param update on the Autopilot Manager
    // instead of running at every loop update
    updateParameters();

    // TODO: reinstantiate _mapper a after parameter update

    // Only process the data to build a landing map when the Autopilot Manager is enabled, Safe Landing is set as the
    // decision maker, and the OA interface is enabled (i.e. the user has activated Safe Landing).
    const bool should_build_landing_map = _landing_manager_config.autopilot_manager_enabled &&
                                          _landing_manager_config.safe_landing_enabled &&
                                          is_obstacle_avoidance_enabled();

    if (should_build_landing_map) {
        timing_tools::Timer timer_mapper("mapper: total", true);

        // Here we capture the downsampled depth data computed in the SensorManager
        std::shared_ptr<ExtendedDownsampledImageF> depth_msg = _downsampled_depth_update_callback();

        const bool is_landing_mapper_healthy = healthCheck(depth_msg);

        if (depth_msg != nullptr && is_landing_mapper_healthy &&
            depth_msg->downsampled_image.depth_pixel_array.size() > 0) {
            const RectifiedIntrinsicsF intrinsics = depth_msg->downsampled_image.intrinsics;
            const DepthPixelArrayF depth_pixel_array = depth_msg->downsampled_image.depth_pixel_array;
            const rclcpp::Time timestamp(depth_msg->timestamp_ns);
            const Eigen::Vector3f position = depth_msg->position;
            const Eigen::Quaternionf orientation = depth_msg->orientation;

            _mapper->updateVehiclePosition(position);
            _mapper->updateVehicleOrientation(orientation);

            timing_tools::Timer timer_pointcloud("point cloud: total     ", true);
            {
                std::lock_guard<std::mutex> lock(_map_mutex);
                _pointcloud_for_mapper.clear();
                _visualizer->prepare_point_cloud_msg(depth_msg->timestamp_ns, intrinsics.rw, intrinsics.rh, _visualize);

                const Eigen::Vector2f principal_point = intrinsics.principal_point();
                const Eigen::Vector2f inverse_focal_length = intrinsics.inverse_focal_length();

                timing_tools::Timer timer_pointcloud_depth_to_3D("point cloud: depth->3D ", true);
                float point_height_min = std::numeric_limits<float>::max();
                for (const DepthPixelF& depth_pixel : depth_pixel_array) {
                    const float depth = depth_pixel.depth;

                    // TODO those ROS2 paramters
                    const float min_depth_to_use = 0.7f;
                    const float max_depth_to_use = 16.f;
                    if (std::isfinite(depth) && (depth > min_depth_to_use)) {
                        Eigen::Matrix<float, 3, 1> point(0.0, 0.0, depth);
                        point.head<2>() = (Eigen::Matrix<float, 2, 1>(depth_pixel.x, depth_pixel.y) - principal_point)
                                              .cwiseProduct(inverse_focal_length) *
                                          depth;
                        point = orientation * point + position;

                        if (depth < max_depth_to_use) {
                            _pointcloud_for_mapper.push_back(point);
                            _visualizer->add_point_to_point_cloud(point, _visualize);
                        }

                        const float point_height = point(2) - position(2);
                        if (point_height < point_height_min) {
                            point_height_min = point_height;
                        }
                    }
                }
                timer_pointcloud_depth_to_3D.stop();

                timing_tools::Timer timer_pointcloud_map_update("point cloud: map update", true);
                _mapper->updateCloud(_pointcloud_for_mapper);
                _mapper->setImageHeightEstimate(point_height_min);
                timer_pointcloud_map_update.stop();

                _visualizer->visualizePointCloud(_visualize);
            }
            timer_pointcloud.stop();

            _images_processed++;
            _points_processed += _pointcloud_for_mapper.size();
            _points_received += intrinsics.rw * intrinsics.rh;

            // Find plain ground
            timing_tools::Timer timer_check_landing_area("check landing area", true);
            Eigen::Vector3f ground_position;
            const landing_mapper::eLandingMapperState state = _mapper->checkLandingArea(ground_position);
            const float height_above_obstacle = _mapper->getHeightAboveObstacle();
            timer_check_landing_area.stop();

            {
                std::lock_guard<std::mutex> lock(_landing_manager_mutex);
                _state = state;
                _height_above_obstacle = height_above_obstacle;
            }

            // Publish the estimated height above obstacle
            auto height_above_obstacle_msg = std_msgs::msg::Float32();
            height_above_obstacle_msg.data = height_above_obstacle;
            _height_above_obstacle_pub->publish(height_above_obstacle_msg);

            // Publish ground height stats
            const height_map::HeightMapStats height_stats = _mapper->getHeightStats();
            publishHeightStats(height_stats);

            // Show result
            visualizeResult(state, ground_position, now());
            visualizeGroundPlane(height_stats.slope_normal, ground_position, now());

        } else if (!is_landing_mapper_healthy) {
            std::lock_guard<std::mutex> lock(_landing_manager_mutex);
            _state = landing_mapper::eLandingMapperState::UNHEALTHY;
        } else {
            std::lock_guard<std::mutex> lock(_landing_manager_mutex);
            if (_state != landing_mapper::eLandingMapperState::CLOSE_TO_GROUND) {
                _state = landing_mapper::eLandingMapperState::UNKNOWN;
            }
        }

        // Show result
        // std::cout << landingManagerOut << " state " << landing_mapper::string_state(_state) << std::endl;

        // Always publish the landing state to the ROS side
        {
            auto landing_state_msg = std_msgs::msg::String();

            std::lock_guard<std::mutex> lock(_landing_manager_mutex);
            landing_state_msg.data = landing_mapper::string_state(_state);

            _landing_state_pub->publish(landing_state_msg);
        }

        timer_mapper.stop();
    }
}

void LandingManager::publishHeightStats(const height_map::HeightMapStats& height_stats) const {
    auto stats_msg = std_msgs::msg::Float32();

    stats_msg.data = height_stats.valid_samples * 100. / height_stats.samples;
    _valid_sample_percentage_pub->publish(stats_msg);

    stats_msg.data = height_stats.slope_deg;
    _slope_angle_pub->publish(stats_msg);

    stats_msg.data = height_stats.above_plane_max_deviation_m;
    _above_plane_max_deviation_pub->publish(stats_msg);

    stats_msg.data = height_stats.below_plane_max_deviation_m;
    _below_plane_max_deviation_pub->publish(stats_msg);

    stats_msg.data = height_stats.std_dev_from_plane_m;
    _std_dev_from_plane_pub->publish(stats_msg);
}

void LandingManager::visualizeResult(landing_mapper::eLandingMapperState state, const Eigen::Vector3f& position,
                                     const rclcpp::Time& timestamp) {
    Eigen::Vector3f vis_position(position[0], position[1], position[2] - 1.0);
    if (state == landing_mapper::eLandingMapperState::CAN_LAND) {
        _visualizer->publishSafeLand(vis_position, timestamp, _mapper_parameter.window_size_m, _visualize);
    } else if (state == landing_mapper::eLandingMapperState::CLOSE_TO_GROUND) {
        _visualizer->publishCloseGround(vis_position, timestamp, _mapper_parameter.window_size_m, _visualize);
    } else {
        _visualizer->publishGround(vis_position, timestamp, _mapper_parameter.window_size_m, _visualize);
    }
}

void LandingManager::visualizeGroundPlane(const Eigen::Vector3f& normal, const Eigen::Vector3f& position,
                                          const rclcpp::Time& timestamp) {
    _visualizer->visualizeGroundPlane(normal, position, timestamp, _mapper_parameter.window_size_m, _visualize);
}

void LandingManager::visualizeMap() {
    _frequency_visualise_map.tic();
    timing_tools::Timer timer_visualise_map("visualise map", true);
    _visualizer->visualizeHeightMap(_mapper->getHeightMap(), now(), _visualize);
    timer_visualise_map.stop();
}

void LandingManager::printStats() {
    std::stringstream ss;

    // Timing stats
    timing_tools::printTimers(ss);
    timing_tools::printFrequencies(ss);

    // Image processing stats
    static constexpr size_t width = 10;
    float points_per_image = 0.;
    if (_images_processed) {
        points_per_image = 1. * _points_processed / _images_processed;
    }
    int percent_points = 0.;
    if (_points_received) {
        percent_points = 100. * _points_processed / _points_received;
    }
    ss << "=== Image processing statistics ===" << std::endl;
    ss << "Images processed" << std::setw(width) << _images_processed << std::endl;
    ss << "Points processed" << std::setw(width) << _points_processed << std::endl;
    ss << "Points / image  " << std::setw(width) << points_per_image << " (" << percent_points << "%)" << std::endl;

    std::cout << std::endl << ss.str() << std::endl;

    // Reset
    timing_tools::resetTimingStatistics();
    timing_tools::resetFrequencyStatistics();
    _images_processed = 0;
    _points_processed = 0;
    _points_received = 0;
}
