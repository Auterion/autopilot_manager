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

#include <MapVisualizer.hpp>

namespace viz {

MapVisualizer::MapVisualizer(rclcpp::Node* node)
    : _node(node),
      marker_pub_(_node->create_publisher<visualization_msgs::msg::MarkerArray>("planner_marker", 1)),
      marker_map_pub_(_node->create_publisher<visualization_msgs::msg::MarkerArray>("map_marker", 1)),
      point_cloud_pub_(_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1)) {}

void MapVisualizer::prepare_point_cloud_msg(int64_t timestamp_ns, size_t width, size_t height, bool enabled) {
    if (!enabled) {
        return;
    }

    if (_cloud_ros2_msg == nullptr) {
        _cloud_ros2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        _cloud_ros2_msg->header.frame_id = NED_FRAME;
        _cloud_ros2_msg->header.stamp = rclcpp::Time(timestamp_ns);

        _cloud_ros2_msg->is_dense = false;
        _cloud_ros2_msg->is_bigendian = false;
    }

    _cloud_ros2_msg->height = height;
    _cloud_ros2_msg->width = width;
    _cloud_ros2_msg->fields.clear();
    _cloud_ros2_msg->fields.reserve(1);
    _cloud_ros2_msg->data.clear();

    sensor_msgs::PointCloud2Modifier pcd_modifier(*_cloud_ros2_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    _pc_iter_x = std::make_shared<sensor_msgs::PointCloud2Iterator<float>>(*_cloud_ros2_msg, "x");
    _pc_iter_y = std::make_shared<sensor_msgs::PointCloud2Iterator<float>>(*_cloud_ros2_msg, "y");
    _pc_iter_z = std::make_shared<sensor_msgs::PointCloud2Iterator<float>>(*_cloud_ros2_msg, "z");
}

void MapVisualizer::visualizePointCloud(bool enabled) {
    if (!enabled) {
        return;
    }

    point_cloud_pub_->publish(*_cloud_ros2_msg);
}

void MapVisualizer::publishSphere(const geometry_msgs::msg::Point& point, std_msgs::msg::ColorRGBA& color,
                                  const geometry_msgs::msg::Vector3& scale, const rclcpp::Time& timestamp,
                                  bool enabled) const {
    if (!enabled) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_goal;
    visualization_msgs::msg::Marker m;

    m.header.frame_id = NED_FRAME;
    m.header.stamp = timestamp;
    m.ns = "goal_position";
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale = scale;
    m.color = color;
    m.lifetime = rclcpp::Duration(5, 0);
    m.id = 0;
    m.pose.position = point;
    marker_goal.markers.push_back(m);
    marker_pub_->publish(marker_goal);
}

std::tuple<float, float, float> MapVisualizer::HSVtoRGB(std::tuple<float, float, float> hsv) {
    std::tuple<float, float, float> rgb;
    float fC = std::get<2>(hsv) * std::get<1>(hsv);  // fV * fS;  // Chroma
    float fHPrime = fmod(std::get<0>(hsv) / 60.0, 6);
    float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
    float fM = std::get<2>(hsv) - fC;

    if (0 <= fHPrime && fHPrime < 1) {
        std::get<0>(rgb) = fC;
        std::get<1>(rgb) = fX;
        std::get<2>(rgb) = 0;
    } else if (1 <= fHPrime && fHPrime < 2) {
        std::get<0>(rgb) = fX;
        std::get<1>(rgb) = fC;
        std::get<2>(rgb) = 0;
    } else if (2 <= fHPrime && fHPrime < 3) {
        std::get<0>(rgb) = 0;
        std::get<1>(rgb) = fC;
        std::get<2>(rgb) = fX;
    } else if (3 <= fHPrime && fHPrime < 4) {
        std::get<0>(rgb) = 0;
        std::get<1>(rgb) = fX;
        std::get<2>(rgb) = fC;
    } else if (4 <= fHPrime && fHPrime < 5) {
        std::get<0>(rgb) = fX;
        std::get<1>(rgb) = 0;
        std::get<2>(rgb) = fC;
    } else if (5 <= fHPrime && fHPrime < 6) {
        std::get<0>(rgb) = fC;
        std::get<1>(rgb) = 0;
        std::get<2>(rgb) = fX;
    } else {
        std::get<0>(rgb) = 0;
        std::get<1>(rgb) = 0;
        std::get<2>(rgb) = 0;
    }

    std::get<0>(rgb) += fM;
    std::get<1>(rgb) += fM;
    std::get<2>(rgb) += fM;

    return rgb;
}

void MapVisualizer::publishVehicle(double safety_radius, const rclcpp::Time& timestamp, bool enabled) {
    if (!enabled) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = BASE_LINK_FRAME;
    m.header.stamp = timestamp;
    m.ns = "vehicle_safety_radius";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 2.0 * safety_radius;
    m.scale.y = 2.0 * safety_radius;
    m.scale.z = 2.0 * safety_radius;
    m.color.a = 0.3;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    marker_array.markers.push_back(m);

    m.ns = "vehicle";
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    constexpr double rotor_diameter = 0.6;
    constexpr double rotor_distance = rotor_diameter * 0.55;
    m.scale.x = rotor_diameter;
    m.scale.y = rotor_diameter;
    m.scale.z = 0.05;
    m.color.a = 0.3;
    m.color.r = 1.0;  // orange
    m.color.g = 0.6;
    m.color.b = 0.0;
    // front left
    m.id++;
    m.pose.position.x = rotor_distance;
    m.pose.position.y = -rotor_distance;
    marker_array.markers.push_back(m);

    // front right
    m.id++;
    m.pose.position.x = rotor_distance;
    m.pose.position.y = rotor_distance;
    marker_array.markers.push_back(m);

    // rear left
    m.id++;
    m.color.r = 0.1;
    m.color.g = 0.1;
    m.color.b = 0.1;
    m.pose.position.x = -rotor_distance;
    m.pose.position.y = -rotor_distance;
    marker_array.markers.push_back(m);

    // rear right
    m.id++;
    m.pose.position.x = -rotor_distance;
    m.pose.position.y = rotor_distance;
    marker_array.markers.push_back(m);

    // TODO: does anyone need rotor arms?
    marker_pub_->publish(marker_array);
}

}  // namespace viz
