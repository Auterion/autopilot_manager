#pragma once

#include <dbus/dbus.h>
#include <mavsdk/mavsdk.h>

#include <AutopilotManagerConfig.hpp>
#include <DbusInterface.hpp>
#include <fstream>
#include <iostream>
#include <landing_manager/LandingManager.hpp>
#include <mission_manager/MissionManager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_manager/SensorManager.hpp>
#include <string>

class AutopilotManager {
   public:
    AutopilotManager(const std::string& mavlinkPort, const std::string& configPath,
                     const std::string& customActionConfigPath);
    ~AutopilotManager();
    auto HandleRequest(DBusMessage* request) -> DBusMessage*;

    mavsdk::Mavsdk _mavsdk_mission_computer;

   private:
    enum ResponseCode { SUCCEED_WITH_COLL_AVOID_OFF = 0, SUCCEED_WITH_COLL_AVOID_ON = 1, FAILED = 2, UNKNOWN = 999 };

    void start();
    void initialProvisioning();

    void run_sensor_manager();
    void run_landing_manager();

    auto SetConfiguration(AutopilotManagerConfig config) -> ResponseCode;
    auto GetConfiguration(AutopilotManagerConfig config) -> ResponseCode;

    uint8_t _autopilot_manager_enabled = false;
    std::string _decision_maker_input_type = "";
    uint8_t _simple_collision_avoid_enabled = false;
    double _simple_collision_avoid_distance_threshold = 0.0;
    std::string _simple_collision_avoid_distance_on_condition_true = "";
    std::string _simple_collision_avoid_distance_on_condition_false = "";

    std::thread _sensor_manager_th;
    std::thread _landing_manager_th;

    std::shared_ptr<MissionManager> _mission_manager;
    std::shared_ptr<SensorManager> _sensor_manager;
    std::shared_ptr<LandingManager> _landing_manager;

    std::mutex _config_mutex;
    std::mutex _distance_to_obstacle_mutex;
    std::mutex _landing_condition_state_mutex;

    std::string _mavlink_port = "14590";
    std::string _config_path = "/shared_container_dir/autopilot_manager.conf";
    std::string _custom_action_config_path =
        "/usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json";

    static constexpr uint8_t kDefaultSystemId = 1;
    static constexpr uint8_t kMavCompIDOnBoardComputer3 = 193;
};
