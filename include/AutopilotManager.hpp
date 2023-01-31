#pragma once

#include <dbus/dbus.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <AutopilotManagerConfig.hpp>
#include <DbusInterface.hpp>
#include <fstream>
#include <iostream>
#include <string>

// Manager modules
#include <collision_avoidance_manager/CollisionAvoidanceManager.hpp>
#include <landing_manager/LandingManager.hpp>
#include <mission_manager/MissionManager.hpp>
#include <sensor_manager/SensorManager.hpp>

static constexpr auto autopilotManagerOut = "[Autopilot Manager] ";

class AutopilotManager {
   public:
    AutopilotManager(const std::string& mavlinkPort, const std::string& configPath,
                     const std::string& customActionConfigPath);
    ~AutopilotManager();
    auto HandleRequest(DBusMessage* request) -> DBusMessage*;

    mavsdk::Mavsdk _mavsdk_mission_computer;

   private:
    enum ResponseCode {
        SUCCEED_WITH_SAFE_LANDING_OFF = 0,
        SUCCEED_WITH_SAFE_LANDING_ON = 1,
        SUCCEED_WITH_COLL_AVOID_OFF = 2,
        SUCCEED_WITH_COLL_AVOID_ON = 3,
        FAILED = 4,
        UNKNOWN = 999
    };

    void initialProvisioning();

    auto SetConfiguration(AutopilotManagerConfig config) -> ResponseCode;
    auto GetConfiguration(AutopilotManagerConfig config) -> ResponseCode;

    void start();

    void start_sensor_manager(std::shared_ptr<mavsdk::System> mavsdk_system);
    void start_collision_avoidance_manager();
    void start_landing_manager(std::shared_ptr<mavsdk::System> mavsdk_system);
    void start_mission_manager(std::shared_ptr<mavsdk::System> mavsdk_system);

    void run_sensor_manager();
    void run_collision_avoidance_manager();
    void run_landing_manager();
    void run_mission_manager();

    void run();
    void update_obstacle_avoidance_enabled();

    void create_avoidance_mavlink_heartbeat_message();

    std::atomic<bool> _obstacle_avoidance_enabled;

    mavlink_message_t _avoidance_heartbeat_message;

    uint8_t _autopilot_manager_enabled = false;
    std::string _decision_maker_input_type = "";

    // General configurations dependent on selected actions
    std::string _script_to_call = "";
    std::string _api_call = "";
    double _local_position_offset_x = 0.0;
    double _local_position_offset_y = 0.0;
    double _local_position_offset_z = 0.0;
    double _local_position_waypoint_x = 0.0;
    double _local_position_waypoint_y = 0.0;
    double _local_position_waypoint_z = 0.0;
    double _global_position_offset_lat = 0.0;
    double _global_position_offset_lon = 0.0;
    double _global_position_offset_alt_amsl = 0.0;
    double _global_position_waypoint_lat = 0.0;
    double _global_position_waypoint_lon = 0.0;
    double _global_position_waypoint_alt_amsl = 0.0;

    // Depth camera configuration
    double _camera_offset_x = 0.0;
    double _camera_offset_y = 0.0;
    double _camera_yaw = 0.0;

    // Safe landing configurations
    uint8_t _safe_landing_enabled = false;
    double _safe_landing_area_square_size = 0.0;
    double _safe_landing_distance_to_ground = 0.0;
    std::string _safe_landing_on_no_safe_land = "";
    uint8_t _safe_landing_try_landing_after_action = false;

    // Landing site search configurations
    double _landing_site_search_speed = 0.0;
    double _landing_site_search_max_distance = 0.0;
    double _landing_site_search_min_height = 0.0;
    double _landing_site_search_min_distance_after_abort = 0.0;
    double _landing_site_search_arrival_radius = 0.0;
    double _landing_site_search_assess_time = 0.0;
    std::string _landing_site_search_strategy = "";

    // Spiral search strategy configuration
    double _landing_site_search_spiral_spacing = 0.0;
    int _landing_site_search_spiral_points = 0;

    // Simple collision avoidance configurations
    uint8_t _simple_collision_avoid_enabled = false;
    double _simple_collision_avoid_distance_threshold = 0.0;
    std::string _simple_collision_avoid_action_on_condition_true = "";

    std::thread _sensor_manager_th;
    std::thread _landing_manager_th;
    std::thread _collision_avoidance_manager_th;
    std::thread _mission_manager_th;

    std::shared_ptr<MissionManager> _mission_manager;
    std::shared_ptr<SensorManager> _sensor_manager;
    std::shared_ptr<LandingManager> _landing_manager;
    std::shared_ptr<CollisionAvoidanceManager> _collision_avoidance_manager;

    std::mutex _config_mutex;
    std::mutex _distance_to_obstacle_mutex;
    std::mutex _downsampled_depth_callback_mutex;
    std::mutex _landing_condition_state_mutex;
    std::mutex _height_above_obstacle_mutex;

    std::string _mavlink_port = "14590";
    std::string _config_path = "/shared_container_dir/autopilot_manager.conf";
    std::string _custom_action_config_path =
        "/usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json";

    std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;

    static constexpr uint8_t kDefaultSystemId = 1;
    static constexpr uint8_t kMavCompIDOnBoardComputer3 = 193;

    std::atomic<bool> _interrupt_received{false};
};
