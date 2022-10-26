#pragma once

#include <dbus/dbus.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class AutopilotManagerConfig {
   public:
    AutopilotManagerConfig() = default;
    ~AutopilotManagerConfig() = default;

    bool AppendToMessage(DBusMessage *reply) const;
    bool InitFromMessage(DBusMessage *request);
    bool WriteToFile(const std::string &config_path) const;
    bool InitFromFile(const std::string &config_path);
    void Print() const;

    uint8_t autopilot_manager_enabled = false;
    std::string decision_maker_input_type = "";

    // General configurations dependent on selected actions
    std::string script_to_call = "";
    std::string api_call = "";
    double local_position_offset_x = 0.0;
    double local_position_offset_y = 0.0;
    double local_position_offset_z = 0.0;
    double local_position_waypoint_x = 0.0;
    double local_position_waypoint_y = 0.0;
    double local_position_waypoint_z = 0.0;
    double global_position_offset_lat = 0.0;
    double global_position_offset_lon = 0.0;
    double global_position_offset_alt_amsl = 0.0;
    double global_position_waypoint_lat = 0.0;
    double global_position_waypoint_lon = 0.0;
    double global_position_waypoint_alt_amsl = 0.0;

    // Depth camera configuration
    double camera_offset_x = 0.0;
    double camera_offset_y = 0.0;
    double camera_yaw = 0.0;

    // Safe landing configurations
    uint8_t safe_landing_enabled = false;
    double safe_landing_area_square_size = 0.0;
    double safe_landing_distance_to_ground = 0.0;
    std::string safe_landing_on_no_safe_land = "";
    uint8_t safe_landing_try_landing_after_action = false;

    // Landing site search configurations
    double landing_site_search_speed = 0.0;
    double landing_site_search_max_distance = 0.0;
    double landing_site_search_min_height = 0.0;
    double landing_site_search_min_distance_after_abort = 0.0;
    double landing_site_search_arrival_radius = 0.0;
    double landing_site_search_assess_time = 0.0;
    std::string landing_site_search_strategy = "";
    // Spiral search config
    double landing_site_search_spiral_spacing = 0.0;
    int landing_site_search_spiral_points = 0;

    // Simple collision avoidance configurations
    uint8_t simple_collision_avoid_enabled = false;
    double simple_collision_avoid_distance_threshold = 0.0;
    std::string simple_collision_avoid_action_on_condition_true = "";
};
