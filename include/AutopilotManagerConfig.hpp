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

    uint8_t autopilot_manager_enabled = false;
    std::string decision_maker_input_type = "";
    uint8_t simple_collision_avoid_enabled = false;
    double simple_collision_avoid_distance_threshold = 0.0;
    std::string simple_collision_avoid_distance_on_condition_true = "";
    std::string simple_collision_avoid_distance_on_condition_false = "";
};
