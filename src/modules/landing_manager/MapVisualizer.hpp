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
#include <landing_mapper/EuclideanSignedDistanceFields.hpp>

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
    void visualizeEsdf(const esdf::EuclideanSignedDistanceFields<T>& esdf, const rclcpp::Time& timestamp,
                       bool enabled = true);

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
void MapVisualizer::visualizeEsdf(const esdf::EuclideanSignedDistanceFields<T>& esdf, const rclcpp::Time& timestamp,
                                  bool enabled) {
    if (!enabled) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker map_marker;
    map_marker.header.frame_id = NED_FRAME;
    map_marker.header.stamp = timestamp;
    map_marker.ns = "esdf";
    map_marker.id = 0;
    map_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    map_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(map_marker);

    const esdfvoxelcube::ESDFVoxelCube esdf_map = esdf.getEsdfMap();
    const double cell_size = esdf_map.getBinEdgeWidth();
    map_marker.header.frame_id = NED_FRAME;
    map_marker.header.stamp = timestamp;
    map_marker.ns = "esdf";
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

    const Eigen::Vector3i grid_min = esdf_map.getLowerBoundaryIndices().array();
    const Eigen::Vector3i grid_max = esdf_map.getUpperBoundaryIndices().array();

    // From previous implementation
    // constexpr float distance_max_value = 20.f;
    // constexpr float distance_min_value = -1.f;
    // constexpr float range_max = 320.f;
    // constexpr float range_min = 0.f;

    for (int i = grid_min[0]; i < grid_max[0]; ++i) {
        for (int j = grid_min[1]; j < grid_max[1]; ++j) {
            for (int k = grid_min[2]; k < grid_max[2]; ++k) {
                const Eigen::Vector3i index(i, j, k);
                const esdfvoxelcube::ESDFNode<T>& voxel = esdf_map.getNode(index);
                if (voxel.is_observed && voxel.distance < cell_size) {
                    const Eigen::Matrix<T, 3, 1> voxel_center_pos = esdf_map.getNodeCenter(index);
                    map_marker.points.push_back(toPoint(voxel_center_pos));

                    // const float h = ((range_max - range_min) * (voxel.distance - distance_min_value) /
                    //                 (distance_max_value - distance_min_value)) +
                    //                 range_min;
                    // MA: not entirely sure what this normalization is meant for. Probably not
                    // necessary anymore since we're only displaying voxels that have small
                    // distances. Hence, color by height for now and wrap around every 25m.

                    // TODO make the 25m / 2.0 a parameter somehow. Either dynamic or by ROS parameters
                    const float h = std::abs(std::fmod(voxel_center_pos.z(), 2.0) / 2.0) * 360;

                    std_msgs::msg::ColorRGBA color;
                    color.a = 0.5;
                    std::tie(color.r, color.g, color.b) = HSVtoRGB(std::make_tuple(h, 1.f, 1.f));
                    map_marker.colors.push_back(color);
                }
            }
        }
    }
    marker_array.markers.push_back(map_marker);
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
