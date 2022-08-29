/****************************************************************************
 *
 * Copyright 2021 Auterion AG. All rights reserved.
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
 * @brief Semiglobal planner ROS 2 MapVisualizer class
 * @file semiglobal_planner_visualization.hpp
 * @author Bastian Jäger <bastian@auterion.com>
 */

#pragma once

#include <common.h>

#include <Eigen/Core>
#include <landing_mapper/HeightMap.hpp>

// ROS dependencies
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace viz {

template <class Derived>
static inline geometry_msgs::msg::Point toPoint(const Eigen::MatrixBase<Derived>& ev3) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    geometry_msgs::msg::Point gmp;
    gmp.x = ev3.x();
    gmp.y = ev3.y();
    gmp.z = ev3.z();
    return gmp;
}

class MapVisualizer {
   public:
    MapVisualizer(rclcpp::Node* node);

    template <class Derived>
    void publishCube(const Eigen::MatrixBase<Derived>& point, std_msgs::msg::ColorRGBA& color,
                     const geometry_msgs::msg::Vector3& scale, const rclcpp::Time& timestamp, bool enabled) const;
    void publishCube(const geometry_msgs::msg::Point& point, std_msgs::msg::ColorRGBA& color,
                     const geometry_msgs::msg::Vector3& scale, const rclcpp::Time& timestamp, bool enabled) const;

    template <class Derived>
    void publishSafeLand(const Eigen::MatrixBase<Derived>& point, const rclcpp::Time& timestamp, float size,
                         bool enabled) const;
    template <class Derived>
    void publishCloseGround(const Eigen::MatrixBase<Derived>& point, const rclcpp::Time& timestamp, float size,
                            bool enabled) const;
    template <class Derived>
    void publishGround(const Eigen::MatrixBase<Derived>& point, const rclcpp::Time& timestamp, float size,
                       bool enabled) const;

    template <typename T>
    void visualizeHeightMap(const height_map::HeightMap<T>& height_map, const rclcpp::Time& timestamp, bool enabled);

    template <class Derived>
    void visualizeGroundPlane(const Eigen::MatrixBase<Derived>& normal, const Eigen::MatrixBase<Derived>& position,
                              const rclcpp::Time& timestamp, float size, bool enabled) const;

    void prepare_point_cloud_msg(int64_t timestamp_ns, size_t width, size_t height, bool enabled = true);

    template <typename T>
    void add_point_to_point_cloud(const Eigen::Matrix<T, 3, 1>& point, bool enabled = true);
    void visualizePointCloud(bool enabled = true);

    void publishVehicle(double safety_radius, const rclcpp::Time& timestamp, bool enabled = true);

    void publishMarkerArray(const visualization_msgs::msg::MarkerArray& marker_array) {
        marker_pub_->publish(marker_array);
    }

   private:
    rclcpp::Node* _node;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    sensor_msgs::msg::PointCloud2::SharedPtr _cloud_ros2_msg;
    std::shared_ptr<sensor_msgs::PointCloud2Iterator<float>> _pc_iter_x;
    std::shared_ptr<sensor_msgs::PointCloud2Iterator<float>> _pc_iter_y;
    std::shared_ptr<sensor_msgs::PointCloud2Iterator<float>> _pc_iter_z;

    std::tuple<float, float, float> HSVtoRGB(std::tuple<float, float, float> hsv);
    int path_length_ = 0;
};

template <class Derived>
void MapVisualizer::publishCube(const Eigen::MatrixBase<Derived>& point, std_msgs::msg::ColorRGBA& color,
                                const geometry_msgs::msg::Vector3& scale, const rclcpp::Time& timestamp,
                                bool enabled) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    publishCube(toPoint(point), color, scale, timestamp, enabled);
}

template <class Derived>
void MapVisualizer::publishSafeLand(const Eigen::MatrixBase<Derived>& point, const rclcpp::Time& timestamp, float size,
                                    bool enabled) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    std_msgs::msg::ColorRGBA color;
    color.a = 0.75;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;

    geometry_msgs::msg::Vector3 scale;
    scale.x = size;
    scale.y = size;
    scale.z = 0.1;

    publishCube(toPoint(point), color, scale, timestamp, enabled);
}

template <class Derived>
void MapVisualizer::publishCloseGround(const Eigen::MatrixBase<Derived>& point, const rclcpp::Time& timestamp,
                                       float size, bool enabled) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    std_msgs::msg::ColorRGBA color;
    color.a = 0.75;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;

    geometry_msgs::msg::Vector3 scale;
    scale.x = size;
    scale.y = size;
    scale.z = 0.1;

    publishCube(toPoint(point), color, scale, timestamp, enabled);
}

template <class Derived>
void MapVisualizer::publishGround(const Eigen::MatrixBase<Derived>& point, const rclcpp::Time& timestamp, float size,
                                  bool enabled) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    std_msgs::msg::ColorRGBA color;
    color.a = 0.75;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;

    geometry_msgs::msg::Vector3 scale;
    scale.x = size;
    scale.y = size;
    scale.z = 0.1;

    publishCube(toPoint(point), color, scale, timestamp, enabled);
}

template <typename T>
void MapVisualizer::visualizeHeightMap(const height_map::HeightMap<T>& height_map, const rclcpp::Time& timestamp,
                                       bool enabled) {
    if (!enabled) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker map_marker;
    map_marker.header.frame_id = NED_FRAME;
    map_marker.header.stamp = timestamp;
    map_marker.ns = "height_map";
    map_marker.id = 0;
    map_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    map_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(map_marker);

    const double cell_size = height_map.getBinEdgeWidth();
    map_marker.header.frame_id = NED_FRAME;
    map_marker.header.stamp = timestamp;
    map_marker.ns = "height_map";
    map_marker.id = 0;
    map_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    map_marker.action = visualization_msgs::msg::Marker::ADD;
    map_marker.pose.orientation.x = 0.0;
    map_marker.pose.orientation.y = 0.0;
    map_marker.pose.orientation.z = 0.0;
    map_marker.pose.orientation.w = 1.0;
    map_marker.scale.x = cell_size;
    map_marker.scale.y = cell_size;
    map_marker.scale.z = cell_size;
    map_marker.color.a = 0.5;
    map_marker.color.r = 1.0;
    map_marker.color.g = 0.0;
    map_marker.color.b = 0.0;

    const Eigen::Matrix<T, 2, 1> map_centre = height_map.getCentrePosition();
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> heights = height_map.heights();
    const int map_size_x = heights.rows();
    const int map_size_y = heights.cols();

    for (int x = 0; x < map_size_x; x++) {
        for (int y = 0; y < map_size_y; y++) {
            if (heights(x, y) != std::numeric_limits<T>::max()) {
                const Eigen::Matrix<T, 3, 1> height_pos(cell_size * (x - map_size_x * 0.5) + map_centre(0),
                                                        cell_size * (y - map_size_y * 0.5) + map_centre(1),
                                                        heights(x, y) + cell_size * 0.5);
                map_marker.points.push_back(toPoint(height_pos));

                const float h = std::abs(std::fmod(height_pos.z(), 2.0) / 2.0) * 360;
                std_msgs::msg::ColorRGBA color;
                color.a = 0.5;
                std::tie(color.r, color.g, color.b) = HSVtoRGB(std::make_tuple(h, 1.f, 1.f));
                map_marker.colors.push_back(color);
            }
        }
    }
    marker_array.markers.push_back(map_marker);
    marker_map_pub_->publish(marker_array);
}

template <class Derived>
void MapVisualizer::visualizeGroundPlane(const Eigen::MatrixBase<Derived>& normal,
                                         const Eigen::MatrixBase<Derived>& position, const rclcpp::Time& timestamp,
                                         float size, bool enabled) const {
    if (!enabled) {
        return;
    }

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

    std_msgs::msg::ColorRGBA color;
    color.a = .6;
    color.r = 1.;
    color.g = 1.;
    color.b = 1.;

    geometry_msgs::msg::Vector3 scale;
    scale.x = size;
    scale.y = size;
    scale.z = 0.01;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker m;

    m.header.frame_id = NED_FRAME;
    m.header.stamp = timestamp;
    m.ns = "ground_plane";
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale = scale;
    m.color = color;
    m.lifetime = rclcpp::Duration(5, 0);
    m.id = 0;
    m.pose.position = toPoint(position);

    // Compute orientation in angle-axis form
    const Eigen::Vector3f vertical = {0., 0., -1.};
    const Eigen::Vector3f axis = vertical.cross(normal).normalized();
    const float angle = acos(-normal(2));

    // Convert to quaternion
    const Eigen::Vector3f axis_scaled = axis * sin(angle / 2);
    m.pose.orientation.x = axis_scaled(0);
    m.pose.orientation.y = axis_scaled(1);
    m.pose.orientation.z = axis_scaled(2);
    m.pose.orientation.w = cos(angle / 2);

    marker_array.markers.push_back(m);
    marker_map_pub_->publish(marker_array);
}

template <typename T>
void MapVisualizer::add_point_to_point_cloud(const Eigen::Matrix<T, 3, 1>& point, bool enabled) {
    if (!enabled) {
        return;
    }

    **_pc_iter_x = static_cast<float>(point.x());
    **_pc_iter_y = static_cast<float>(point.y());
    **_pc_iter_z = static_cast<float>(point.z());
    ++(*_pc_iter_x);
    ++(*_pc_iter_y);
    ++(*_pc_iter_z);
}

}  // namespace viz
