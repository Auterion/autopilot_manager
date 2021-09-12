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
 * @brief Mission Manager
 * @file MissionManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <MissionManager.hpp>
#include <atomic>
#include <future>
#include <iostream>
#include <string>
#include <utility>

using namespace std::chrono_literals;

static std::atomic<bool> int_signal{false};

MissionManager::MissionManager(std::shared_ptr<mavsdk::System> mavsdk_system,
                               const std::string& path_to_custom_action_file)
    : _config_update_callback([]() { return MissionManagerConfiguration{}; }),
      _path_to_custom_action_file{std::move(path_to_custom_action_file)},
      _mission_manager_config{},
      _mavsdk_system{std::move(mavsdk_system)},
      _action_triggered{false},
      _in_air{false},
      _landing{false} {}

MissionManager::~MissionManager() { deinit(); }

void MissionManager::init() {
    std::cout << missionManagerOut << " Started!" << std::endl;
    _custom_action_handler = std::make_shared<CustomActionHandler>(_mavsdk_system, _path_to_custom_action_file);

    // Actions are processed and executed in the Mission Manager decion maker
    _action = std::make_shared<mavsdk::Action>(_mavsdk_system);

    // Telemetry data checks are fundamental for proper execution
    _telemetry = std::make_shared<mavsdk::Telemetry>(_mavsdk_system);
}

void MissionManager::deinit() {
    _decision_maker_th.join();
    _custom_action_handler.reset();
}

void MissionManager::run() {
    // Start the desicion maker thread
    _decision_maker_th = std::thread(&MissionManager::decision_maker_run, this);

    // Start custom action handler
    if (_custom_action_handler->start()) {
        _custom_action_handler->run();
    }
}

void MissionManager::handle_safe_landing(std::chrono::time_point<std::chrono::system_clock> now) {
    if (_landing && !_on_ground) {
        if (_landing_condition_state_update_callback()) {  // landing_mapper::eLandingMapperState::CAN_LAND
            std::cout << missionManagerOut << "Able to land" << std::endl;
        } else {
            if (_mission_manager_config.safe_landing_on_no_safe_land == "HOLD") {
                _action->hold();
                std::cout << missionManagerOut << "Position hold triggered" << _action_triggered << std::endl;
            } else if (_mission_manager_config.safe_landing_on_no_safe_land == "RTL") {
                _action->return_to_launch();
                std::cout << missionManagerOut << "RTL triggered" << _action_triggered << std::endl;
            } else if (_mission_manager_config.safe_landing_on_no_safe_land == "MOVE_XYZ_WRT_CURRENT") {
                _action->hold();
                std::cout << missionManagerOut
                          << "MOVE_XYZ_WRT_CURRENT action currently not supported. Holding position..." << std::endl;
            } else if (_mission_manager_config.safe_landing_on_no_safe_land == "GO_TO_WAYPOINT_XYZ") {
                _action->hold();
                std::cout << missionManagerOut
                          << "GO_TO_WAYPOINT_XYZ action currently not supported. Holding position..." << std::endl;
            } else if (_mission_manager_config.safe_landing_on_no_safe_land == "MOVE_LLA_WRT_CURRENT") {
                _action->hold();
                std::cout << missionManagerOut
                          << "MOVE_LLA_WRT_CURRENT action currently not supported. Holding position..." << std::endl;
            } else if (_mission_manager_config.safe_landing_on_no_safe_land == "GO_TO_WAYPOINT") {
                _action->hold();
                std::cout << missionManagerOut << "GO_TO_WAYPOINT action currently not supported. Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.safe_landing_on_no_safe_land == "SCRIPT_CALL") {
                _action->hold();
                std::cout << missionManagerOut << "SCRIPT_CALL action currently not supported. Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.safe_landing_on_no_safe_land == "API_CALL") {
                _action->hold();
                std::cout << missionManagerOut << "API_CALL action currently not supported. Holding position..."
                          << std::endl;
            }

            _action_triggered = true;
            _last_time = now;
        }
    }
}

void MissionManager::handle_simple_collision_avoidance(std::chrono::time_point<std::chrono::system_clock> now) {
    if (_mission_manager_config.simple_collision_avoid_enabled != 0U) {
        // std::cout << "Depth measured: " << _distance_to_obstacle_update_callback()
        //           << " | threshold: " << _mission_manager_config.simple_collision_avoid_distance_threshold
        //           << " | in air: " << _in_air << " | is action triggered? " << std::boolalpha
        //           << _action_triggered << std::endl;

        if (std::isfinite(_distance_to_obstacle_update_callback()) &&
            _distance_to_obstacle_update_callback() <=
                _mission_manager_config.simple_collision_avoid_distance_threshold &&
            _in_air && !_action_triggered) {  // only trigger the condition when the vehicle is in-air
            if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "HOLD") {
                _action->hold();
                std::cout << missionManagerOut << "Position hold triggered" << _action_triggered << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "RTL") {
                _action->return_to_launch();
                std::cout << missionManagerOut << "RTL triggered" << _action_triggered << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "MOVE_XYZ_WRT_CURRENT") {
                _action->hold();
                std::cout << missionManagerOut
                          << "MOVE_XYZ_WRT_CURRENT action currently not supported. Holding position..." << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "GO_TO_WAYPOINT_XYZ") {
                _action->hold();
                std::cout << missionManagerOut
                          << "GO_TO_WAYPOINT_XYZ action currently not supported. Holding position..." << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "MOVE_LLA_WRT_CURRENT") {
                _action->hold();
                std::cout << missionManagerOut
                          << "MOVE_LLA_WRT_CURRENT action currently not supported. Holding position..." << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "GO_TO_WAYPOINT") {
                _action->hold();
                std::cout << missionManagerOut << "GO_TO_WAYPOINT action currently not supported. Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "LAND") {
                _action->land();
                std::cout << missionManagerOut << "Land triggered" << _action_triggered << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "SCRIPT_CALL") {
                _action->hold();
                std::cout << missionManagerOut << "SCRIPT_CALL action currently not supported. Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "API_CALL") {
                _action->hold();
                std::cout << missionManagerOut << "API_CALL action currently not supported. Holding position..."
                          << std::endl;
            }

            _action_triggered = true;
            _last_time = now;
        }
    }
}

void MissionManager::decision_maker_run() {
    // Init action trigger timer
    _last_time = std::chrono::system_clock::now();

    // Get the landing state so we know when the vehicle is in-air
    _telemetry->subscribe_landed_state([this](mavsdk::Telemetry::LandedState landed_state) {
        switch (landed_state) {
            case mavsdk::Telemetry::LandedState::InAir:
                _in_air = true;
                _landing = false;
                _on_ground = false;
                break;
            case mavsdk::Telemetry::LandedState::Landing:
                _in_air = false;
                _landing = true;
                _on_ground = false;
                break;
            case mavsdk::Telemetry::LandedState::Unknown:
            case mavsdk::Telemetry::LandedState::OnGround:
                _in_air = false;
                _landing = false;
                _on_ground = true;
                break;
            case mavsdk::Telemetry::LandedState::TakingOff:
            default:
                _in_air = false;
                _landing = false;
                _on_ground = false;
        }
    });

    while (!int_signal) {
        // Update configuration at each iteration
        _mission_manager_config = _config_update_callback();

        auto now = std::chrono::system_clock::now();

        if (_mission_manager_config.decision_maker_input_type == "SAFE_LANDING") {
            handle_safe_landing(now);
        } else if (_mission_manager_config.decision_maker_input_type == "SIMPLE_COLLISION_AVOIDANCE") {
            handle_simple_collision_avoidance(now);
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_time).count() >= 5000) {
            _action_triggered = false;
        }

        // Decision maker runs at 20hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

CustomActionHandler::CustomActionHandler(std::shared_ptr<mavsdk::System> mavsdk_system,
                                         const std::string& path_to_custom_action_file)
    : _mavsdk_system{std::move(mavsdk_system)}, _path_to_custom_action_file{std::move(path_to_custom_action_file)} {}

CustomActionHandler::~CustomActionHandler() { int_signal.store(true, std::memory_order_relaxed); }

auto CustomActionHandler::start() -> bool {
    if (_mavsdk_system->has_autopilot()) {
        // Custom actions are processed and executed in the Mission Manager
        _custom_action = std::make_shared<mavsdk::CustomAction>(_mavsdk_system);

        return true;
    }
    return false;
}

auto CustomActionHandler::run() -> void {
    std::cout << missionManagerOut << " System ready! Waiting for custom actions to process..." << std::endl;

    // Subscribe to the cancelation message
    _custom_action->subscribe_custom_action_cancellation(
        [this](bool canceled) { _action_stopped.store(canceled, std::memory_order_relaxed); });

    auto new_actions_check_th = std::thread(&CustomActionHandler::new_action_check, this);
    new_actions_check_th.detach();

    auto new_action_time = std::chrono::system_clock::now() + std::chrono::minutes(1);  // dummy init
    auto start_time = new_action_time;

    // Get the custom action to process
    _custom_action->subscribe_custom_action(
        [this, &start_time, &new_action_time](mavsdk::CustomAction::ActionToExecute action_to_exec) {
            if (_actions.empty() || _actions.back().id != action_to_exec.id) {
                auto now = std::chrono::system_clock::now();
                // This is a safeguard and a workaround in case the FMU sends consecutive MAV_CMDs because
                // he didn't get an ACK
                if (new_action_time == start_time ||
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - new_action_time).count() >= 1500) {
                    _actions.push_back(action_to_exec);
                    _new_action.store(true, std::memory_order_relaxed);
                    std::cout << missionManagerOut << " New action received with ID " << action_to_exec.id << std::endl;
                    new_action_time = now;
                }
            }
        });
}

void CustomActionHandler::send_progress_status(mavsdk::CustomAction::ActionToExecute action) {
    while (!_action_stopped.load() && _actions_result.back() != mavsdk::CustomAction::Result::Unknown) {
        mavsdk::CustomAction::ActionToExecute action_exec{};
        action_exec.id = action.id;
        action_exec.progress = _actions_progress.back();
        auto action_result = _actions_result.back();

        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        // Send response with the result and the progress
        _custom_action->respond_custom_action_async(
            action_exec, action_result, [&prom](mavsdk::CustomAction::Result /*result*/) { prom.set_value(); });

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    };
}

void CustomActionHandler::new_action_check() {
    while (!int_signal) {
        if (_new_action.load()) {
            process_custom_action(_actions.back());
            _new_action.store(false, std::memory_order_relaxed);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void CustomActionHandler::process_custom_action(mavsdk::CustomAction::ActionToExecute action) {
    std::cout << missionManagerOut << " Custom action #" << action.id << " being processed" << std::endl;

    // Get the custom action metadata
    std::promise<mavsdk::CustomAction::ActionMetadata> prom;
    std::future<mavsdk::CustomAction::ActionMetadata> fut = prom.get_future();
    _custom_action->custom_action_metadata_async(
        action, _path_to_custom_action_file,
        [&prom](mavsdk::CustomAction::Result /*result*/, const mavsdk::CustomAction::ActionMetadata& action_metadata) {
            prom.set_value(action_metadata);
        });

    _actions_metadata.push_back(fut.get());

    // Start
    _actions_result.push_back(mavsdk::CustomAction::Result::InProgress);
    _actions_progress.push_back(0.0);
    std::cout << missionManagerOut << " Custom action #" << _actions_metadata.back().id
              << " current progress: " << _actions_progress.back() << "%" << std::endl;

    // Start the progress status report thread
    _progress_threads.push_back(std::thread(&CustomActionHandler::send_progress_status, this, _actions.back()));

    // Start the custom action execution
    // For the purpose of the test, we are storing all the actions but only processing the last one.
    // This means that only one action at a time can be processed
    execute_custom_action(_actions_metadata.back());

    if (_actions_progress.back() == 100.0 && _actions_result.back() == mavsdk::CustomAction::Result::Success) {
        // Used to stop the progress status thread
        _actions_result.back() = mavsdk::CustomAction::Result::Unknown;
    }

    _progress_threads.back().join();
}

void CustomActionHandler::execute_custom_action(const mavsdk::CustomAction::ActionMetadata& action_metadata) {
    if (!action_metadata.stages.empty()) {
        for (unsigned i = 0; i < action_metadata.stages.size(); i++) {
            if (!_action_stopped.load()) {
                mavsdk::CustomAction::Result stage_res =
                    _custom_action->execute_custom_action_stage(action_metadata.stages[i]);
                (void)stage_res;
            }

            auto wait_time = action_metadata.stages[i].timestamp_stop * 1s;
            std::unique_lock<std::mutex> lock(cancel_mtx);
            cancel_signal.wait_for(lock, wait_time, [this]() { return _action_stopped.load(); });

            if (!_action_stopped.load()) {
                _actions_progress.back() = (i + 1.0) / action_metadata.stages.size() * 100.0;

                if (_actions_progress.back() != 100.0) {
                    _actions_result.back() = mavsdk::CustomAction::Result::InProgress;
                    std::cout << missionManagerOut << " Custom action #" << _actions_metadata.back().id
                              << " current progress: " << _actions_progress.back() << "%" << std::endl;
                } else {
                    _actions_result.back() = mavsdk::CustomAction::Result::Success;
                }
            }
        }

    } else if (!action_metadata.global_script.empty()) {
        if (!_action_stopped.load()) {
            std::promise<mavsdk::CustomAction::Result> prom;
            std::future<mavsdk::CustomAction::Result> fut = prom.get_future();
            _custom_action->execute_custom_action_global_script_async(
                action_metadata.global_script,
                [&prom](mavsdk::CustomAction::Result script_result) { prom.set_value(script_result); });

            if (!std::isnan(action_metadata.global_timeout)) {
                std::chrono::seconds timeout(static_cast<long int>(action_metadata.global_timeout));
                while (fut.wait_for(timeout) != std::future_status::ready) {
                };
            }

            _actions_result.back() = fut.get();

            if (_actions_result.back() == mavsdk::CustomAction::Result::Success) {
                _actions_progress.back() = 100.0;
            }
        }
    }

    // We wait for half a second to make sure that the ACCEPTED ACKs are sent
    // to the FMU and don't get lost
    auto wait_time = 500ms;
    std::unique_lock<std::mutex> lock(cancel_mtx);
    cancel_signal.wait_for(lock, wait_time, [this]() { return _action_stopped.load(); });

    if (_action_stopped.load()) {
        std::cout << missionManagerOut << " Custom action #" << _actions_metadata.back().id << " canceled!"
                  << std::endl;
        _action_stopped.store(false, std::memory_order_relaxed);
    } else if (!_action_stopped.load() && _actions_progress.back() == 100) {
        std::cout << missionManagerOut << " Custom action #" << _actions_metadata.back().id << " executed!"
                  << std::endl;
    }

    // clear actions after they are processed
    _actions.clear();
}
