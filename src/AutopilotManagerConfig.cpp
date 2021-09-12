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
 * @brief AutopilotManagerConfig
 * @file AutopilotManagerConfig.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <AutopilotManagerConfig.hpp>

bool AutopilotManagerConfig::InitFromMessage(DBusMessage* request) {
    if (request != nullptr) {
        DBusError error;
        dbus_error_init(&error);
        char* returnedDecisionMakerType = nullptr;
        char* returnedScriptToCall = nullptr;
        char* returnedAPICall = nullptr;
        char* returnedSafeLandingOnNoSafeLand = nullptr;
        char* returnedCollAvoidOnCondTrue = nullptr;
        std::cout << "[AutopilotManagerConfig] Parse response..." << std::endl;
        dbus_message_get_args(
            request, &error,
            // General parameters
            DBUS_TYPE_UINT32, &autopilot_manager_enabled, DBUS_TYPE_STRING, &returnedDecisionMakerType,
            // General configurations dependent on selected actions
            DBUS_TYPE_STRING, &returnedScriptToCall, DBUS_TYPE_STRING, &returnedAPICall, DBUS_TYPE_DOUBLE,
            &local_position_offset_x, DBUS_TYPE_DOUBLE, &local_position_offset_y, DBUS_TYPE_DOUBLE,
            &local_position_offset_z, DBUS_TYPE_DOUBLE, &local_position_waypoint_x, DBUS_TYPE_DOUBLE,
            &local_position_waypoint_y, DBUS_TYPE_DOUBLE, &local_position_waypoint_z, DBUS_TYPE_DOUBLE,
            &global_position_offset_lat, DBUS_TYPE_DOUBLE, &global_position_offset_lon, DBUS_TYPE_DOUBLE,
            &global_position_offset_alt_amsl, DBUS_TYPE_DOUBLE, &global_position_waypoint_lat, DBUS_TYPE_DOUBLE,
            &global_position_waypoint_lon, DBUS_TYPE_DOUBLE, &global_position_waypoint_alt_amsl,
            // Safe Landing configurations
            DBUS_TYPE_UINT32, &safe_landing_enabled, DBUS_TYPE_DOUBLE, &safe_landing_area_square_size, DBUS_TYPE_DOUBLE,
            &safe_landing_distance_to_ground, DBUS_TYPE_STRING, &returnedSafeLandingOnNoSafeLand, DBUS_TYPE_UINT32,
            &safe_landing_try_landing_after_action,
            // Simple collision avoidance configurations
            DBUS_TYPE_UINT32, &simple_collision_avoid_enabled, DBUS_TYPE_DOUBLE,
            &simple_collision_avoid_distance_threshold, DBUS_TYPE_STRING, &returnedCollAvoidOnCondTrue,
            DBUS_TYPE_INVALID);
        if (dbus_error_is_set(&error)) {
            std::cerr << "[AutopilotManagerConfig] Failed getting message arguments: " << error.message << std::endl;
            dbus_error_free(&error);
        } else {
            decision_maker_input_type = std::string(returnedDecisionMakerType);
            script_to_call = std::string(returnedScriptToCall);
            api_call = std::string(returnedAPICall);
            safe_landing_on_no_safe_land = std::string(returnedSafeLandingOnNoSafeLand);
            simple_collision_avoid_action_on_condition_true = std::string(returnedCollAvoidOnCondTrue);

            std::cout << "[AutopilotManagerConfig] Received: [" << std::endl;
            std::cout << "    autopilot_manager_enabled: " << std::boolalpha
                      << std::to_string(autopilot_manager_enabled) << std::endl;
            std::cout << "    decision_maker_input_type: " << decision_maker_input_type << std::endl;

            std::cout << "    script_to_call: " << script_to_call << std::endl;
            std::cout << "    api_call: " << api_call << std::endl;
            std::cout << "    local_position_offset_x: " << local_position_offset_x << std::endl;
            std::cout << "    local_position_offset_y: " << local_position_offset_y << std::endl;
            std::cout << "    local_position_offset_z: " << local_position_offset_z << std::endl;
            std::cout << "    local_position_waypoint_x: " << local_position_waypoint_x << std::endl;
            std::cout << "    local_position_waypoint_y: " << local_position_waypoint_y << std::endl;
            std::cout << "    local_position_waypoint_z: " << local_position_waypoint_z << std::endl;
            std::cout << "    global_position_offset_lat: " << global_position_offset_lat << std::endl;
            std::cout << "    global_position_offset_lon: " << global_position_offset_lon << std::endl;
            std::cout << "    global_position_offset_alt_amsl: " << global_position_offset_alt_amsl << std::endl;
            std::cout << "    global_position_waypoint_lat: " << global_position_waypoint_lat << std::endl;
            std::cout << "    global_position_waypoint_lon: " << global_position_waypoint_lon << std::endl;
            std::cout << "    global_position_waypoint_alt_amsl: " << global_position_waypoint_alt_amsl << std::endl;

            std::cout << "    safe_landing_enabled: " << std::boolalpha << std::to_string(safe_landing_enabled)
                      << std::endl;
            std::cout << "    safe_landing_area_square_size: " << safe_landing_area_square_size << std::endl;
            std::cout << "    safe_landing_distance_to_ground: " << safe_landing_distance_to_ground << std::endl;
            std::cout << "    safe_landing_on_no_safe_land: " << safe_landing_on_no_safe_land << std::endl;
            std::cout << "    safe_landing_try_landing_after_action: " << std::boolalpha
                      << std::to_string(safe_landing_try_landing_after_action) << std::endl;

            std::cout << "    simple_collision_avoid_enabled: " << std::boolalpha
                      << std::to_string(simple_collision_avoid_enabled) << std::endl;
            std::cout << "    simple_collision_avoid_distance_threshold: " << simple_collision_avoid_distance_threshold
                      << std::endl;
            std::cout << "    simple_collision_avoid_action_on_condition_true: "
                      << simple_collision_avoid_action_on_condition_true << std::endl;
            std::cout << "]" << std::endl;
            return true;
        }
    }
    return false;
}

bool AutopilotManagerConfig::AppendToMessage(DBusMessage* reply) const {
    if (reply != nullptr) {
        dbus_message_append_args(
            reply,
            // General parameters
            DBUS_TYPE_UINT32, &autopilot_manager_enabled, DBUS_TYPE_STRING, &decision_maker_input_type,
            // General configurations dependent on selected actions
            DBUS_TYPE_STRING, &script_to_call, DBUS_TYPE_STRING, &api_call, DBUS_TYPE_DOUBLE, &local_position_offset_x,
            DBUS_TYPE_DOUBLE, &local_position_offset_y, DBUS_TYPE_DOUBLE, &local_position_offset_z, DBUS_TYPE_DOUBLE,
            &local_position_waypoint_x, DBUS_TYPE_DOUBLE, &local_position_waypoint_y, DBUS_TYPE_DOUBLE,
            &local_position_waypoint_z, DBUS_TYPE_DOUBLE, &global_position_offset_lat, DBUS_TYPE_DOUBLE,
            &global_position_offset_lon, DBUS_TYPE_DOUBLE, &global_position_offset_alt_amsl, DBUS_TYPE_DOUBLE,
            &global_position_waypoint_lat, DBUS_TYPE_DOUBLE, &global_position_waypoint_lon, DBUS_TYPE_DOUBLE,
            &global_position_waypoint_alt_amsl,
            // Safe Landing configurations
            DBUS_TYPE_UINT32, &safe_landing_enabled, DBUS_TYPE_DOUBLE, &safe_landing_area_square_size, DBUS_TYPE_DOUBLE,
            &safe_landing_distance_to_ground, DBUS_TYPE_STRING, &safe_landing_on_no_safe_land, DBUS_TYPE_UINT32,
            &safe_landing_try_landing_after_action,
            // Simple collision avoidance configurations
            DBUS_TYPE_UINT32, &simple_collision_avoid_enabled, DBUS_TYPE_DOUBLE,
            &simple_collision_avoid_distance_threshold, DBUS_TYPE_STRING,
            &simple_collision_avoid_action_on_condition_true, DBUS_TYPE_INVALID);
        return true;
    }
    return false;
}

bool AutopilotManagerConfig::WriteToFile(const std::string& config_path) const {
    const std::string temp_path = config_path + ".tmp";
    std::ofstream file(temp_path);
    if (file.is_open()) {
        file << "[AutopilotManagerConfig]" << std::endl;

        file << "autopilot_manager_enabled=" << std::to_string(autopilot_manager_enabled) << std::endl;
        file << "decision_maker_input_type=" << decision_maker_input_type << std::endl;

        file << "script_to_call=" << script_to_call << std::endl;
        file << "api_call=" << api_call << std::endl;
        file << "local_position_offset_x=" << std::to_string(local_position_offset_x) << std::endl;
        file << "local_position_offset_y=" << std::to_string(local_position_offset_y) << std::endl;
        file << "local_position_offset_z=" << std::to_string(local_position_offset_z) << std::endl;
        file << "local_position_waypoint_x=" << std::to_string(local_position_waypoint_x) << std::endl;
        file << "local_position_waypoint_y=" << std::to_string(local_position_waypoint_y) << std::endl;
        file << "local_position_waypoint_z=" << std::to_string(local_position_waypoint_z) << std::endl;
        file << "global_position_offset_lat=" << std::to_string(global_position_offset_lat) << std::endl;
        file << "global_position_offset_lon=" << std::to_string(global_position_offset_lon) << std::endl;
        file << "global_position_offset_alt_amsl=" << std::to_string(global_position_offset_alt_amsl) << std::endl;
        file << "global_position_waypoint_lat=" << std::to_string(global_position_waypoint_lat) << std::endl;
        file << "global_position_waypoint_lon=" << std::to_string(global_position_waypoint_lon) << std::endl;
        file << "global_position_waypoint_alt_amsl=" << std::to_string(global_position_waypoint_alt_amsl) << std::endl;

        file << "safe_landing_enabled=" << std::to_string(simple_collision_avoid_enabled) << std::endl;
        file << "safe_landing_area_square_size=" << std::to_string(safe_landing_area_square_size) << std::endl;
        file << "safe_landing_distance_to_ground=" << std::to_string(safe_landing_distance_to_ground) << std::endl;
        file << "safe_landing_on_no_safe_land=" << safe_landing_on_no_safe_land << std::endl;
        file << "safe_landing_try_landing_after_action=" << std::to_string(safe_landing_try_landing_after_action)
             << std::endl;

        file << "simple_collision_avoid_enabled=" << std::to_string(simple_collision_avoid_enabled) << std::endl;
        file << "simple_collision_avoid_distance_threshold="
             << std::to_string(simple_collision_avoid_distance_threshold) << std::endl;
        file << "simple_collision_avoid_action_on_condition_true=" << simple_collision_avoid_action_on_condition_true
             << std::endl;

        file.close();
        if (rename(temp_path.c_str(), config_path.c_str()) != 0) {
            std::cerr << "[AutopilotManagerConfig] Unable to write file to: " << config_path << std::endl;
            return false;
        }
        return true;
    } else {
        std::cerr << "[AutopilotManagerConfig] Unable to open file: " << config_path << std::endl;
        return false;
    }
}

bool AutopilotManagerConfig::InitFromFile(const std::string& config_path) {
    std::ifstream file(config_path);
    if (file.is_open()) {
        std::istringstream sin;
        std::string line;
        std::cout << "[AutopilotManagerConfig] Loaded config from file:" << std::endl;
        while (std::getline(file, line)) {
            sin.str(line.substr(line.find("=") + 1));
            if (line.find("autopilot_manager_enabled") != std::string::npos) {
                std::cout << "\tautopilot_manager_enabled: " << sin.str() << std::endl;
                autopilot_manager_enabled = std::stoi(sin.str());
            }
            if (line.find("decision_maker_input_type") != std::string::npos) {
                std::cout << "\tdecision_maker_input_type: " << sin.str() << std::endl;
                decision_maker_input_type = sin.str();
            }

            if (line.find("script_to_call") != std::string::npos) {
                std::cout << "\tscript_to_call: " << sin.str() << std::endl;
                script_to_call = sin.str();
            }
            if (line.find("api_call") != std::string::npos) {
                std::cout << "\tapi_call: " << sin.str() << std::endl;
                api_call = sin.str();
            }
            if (line.find("local_position_offset_x") != std::string::npos) {
                std::cout << "\tlocal_position_offset_x: " << sin.str() << std::endl;
                local_position_offset_x = std::stod(sin.str());
            }
            if (line.find("local_position_offset_y") != std::string::npos) {
                std::cout << "\tlocal_position_offset_y: " << sin.str() << std::endl;
                local_position_offset_y = std::stod(sin.str());
            }
            if (line.find("local_position_offset_z") != std::string::npos) {
                std::cout << "\tlocal_position_offset_z: " << sin.str() << std::endl;
                local_position_offset_z = std::stod(sin.str());
            }
            if (line.find("local_position_waypoint_x") != std::string::npos) {
                std::cout << "\tlocal_position_waypoint_x: " << sin.str() << std::endl;
                local_position_waypoint_x = std::stod(sin.str());
            }
            if (line.find("local_position_waypoint_y") != std::string::npos) {
                std::cout << "\tlocal_position_waypoint_y: " << sin.str() << std::endl;
                local_position_waypoint_y = std::stod(sin.str());
            }
            if (line.find("local_position_waypoint_z") != std::string::npos) {
                std::cout << "\tlocal_position_waypoint_z: " << sin.str() << std::endl;
                local_position_waypoint_z = std::stod(sin.str());
            }
            if (line.find("global_position_offset_lat") != std::string::npos) {
                std::cout << "\tglobal_position_offset_lat: " << sin.str() << std::endl;
                global_position_offset_lat = std::stod(sin.str());
            }
            if (line.find("global_position_offset_lon") != std::string::npos) {
                std::cout << "\tglobal_position_offset_lon: " << sin.str() << std::endl;
                global_position_offset_lon = std::stod(sin.str());
            }
            if (line.find("global_position_offset_alt_amsl") != std::string::npos) {
                std::cout << "\tglobal_position_offset_alt_amsl: " << sin.str() << std::endl;
                global_position_offset_alt_amsl = std::stod(sin.str());
            }
            if (line.find("global_position_waypoint_lat") != std::string::npos) {
                std::cout << "\tglobal_position_waypoint_lat: " << sin.str() << std::endl;
                global_position_waypoint_lat = std::stod(sin.str());
            }
            if (line.find("global_position_waypoint_lon") != std::string::npos) {
                std::cout << "\tglobal_position_waypoint_lon: " << sin.str() << std::endl;
                global_position_waypoint_lon = std::stod(sin.str());
            }
            if (line.find("global_position_waypoint_alt_amsl") != std::string::npos) {
                std::cout << "\tglobal_position_waypoint_alt_amsl: " << sin.str() << std::endl;
                global_position_waypoint_alt_amsl = std::stod(sin.str());
            }

            if (line.find("safe_landing_enabled") != std::string::npos) {
                std::cout << "\tsafe_landing_enabled: " << sin.str() << std::endl;
                safe_landing_enabled = std::stoi(sin.str());
            }
            if (line.find("safe_landing_area_square_size") != std::string::npos) {
                std::cout << "\tsafe_landing_area_square_size: " << sin.str() << std::endl;
                safe_landing_area_square_size = std::stod(sin.str());
            }
            if (line.find("safe_landing_distance_to_ground") != std::string::npos) {
                std::cout << "\tsafe_landing_distance_to_ground: " << sin.str() << std::endl;
                safe_landing_distance_to_ground = std::stod(sin.str());
            }
            if (line.find("safe_landing_on_no_safe_land") != std::string::npos) {
                std::cout << "\tsafe_landing_on_no_safe_land: " << sin.str() << std::endl;
                safe_landing_on_no_safe_land = sin.str();
            }
            if (line.find("safe_landing_try_landing_after_action") != std::string::npos) {
                std::cout << "\tsafe_landing_try_landing_after_action: " << sin.str() << std::endl;
                safe_landing_try_landing_after_action = std::stoi(sin.str());
            }

            if (line.find("simple_collision_avoid_enabled") != std::string::npos) {
                std::cout << "\tsimple_collision_avoid_enabled: " << sin.str() << std::endl;
                simple_collision_avoid_enabled = std::stoi(sin.str());
            }
            if (line.find("simple_collision_avoid_distance_threshold") != std::string::npos) {
                std::cout << "\tsimple_collision_avoid_distance_threshold: " << sin.str() << std::endl;
                simple_collision_avoid_distance_threshold = std::stod(sin.str());
            }
            if (line.find("simple_collision_avoid_action_on_condition_true") != std::string::npos) {
                std::cout << "\tsimple_collision_avoid_action_on_condition_true: " << sin.str() << std::endl;
                simple_collision_avoid_action_on_condition_true = sin.str();
            }
            sin.clear();
        }
        file.close();
        return true;
    } else {
        std::cerr << "[AutopilotManagerConfig] Unable to open file: " << config_path << std::endl;
        return false;
    }
}
