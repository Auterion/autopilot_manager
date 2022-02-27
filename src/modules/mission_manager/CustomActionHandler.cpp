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
 * @brief Custom Action Handler
 * @file CustomActionHandler.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <CustomActionHandler.hpp>

using namespace std::chrono_literals;

static std::atomic<bool> int_signal{false};

CustomActionHandler::CustomActionHandler(std::shared_ptr<mavsdk::System> mavsdk_system,
                                         std::shared_ptr<mavsdk::Telemetry> telemetry,
                                         const std::string& path_to_custom_action_file)
    : _mavsdk_system{std::move(mavsdk_system)},
      _telemetry{std::move(telemetry)},
      _path_to_custom_action_file{std::move(path_to_custom_action_file)} {}

CustomActionHandler::~CustomActionHandler() {
    int_signal.store(true, std::memory_order_relaxed);
    _custom_action.reset();
}

auto CustomActionHandler::start() -> bool {
    if (_mavsdk_system->has_autopilot()) {
        // Custom actions are processed and executed in the Mission Manager
        _custom_action = std::make_shared<mavsdk::CustomAction>(_mavsdk_system);

        // Get the landing state so we can decide if landing or takeoff are complete
        _telemetry->subscribe_landed_state(
            [this](mavsdk::Telemetry::LandedState landed_state) { _landed_state = landed_state; });

        return true;
    }
    return false;
}

auto CustomActionHandler::run() -> void {
    std::cout << customActionHandlerOut << " System ready! Waiting for custom actions to process..." << std::endl;

    // Subscribe to the cancelation message
    _custom_action->subscribe_custom_action_cancellation([this](bool /**/) {
        std::cout << customActionHandlerOut << " Requested action " << _actions_metadata.back().id
                  << " to be cancelled..." << std::endl;
        _action_stopped.store(true, std::memory_order_relaxed);
    });

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
                    std::cout << customActionHandlerOut << " New action received with ID " << action_to_exec.id
                              << std::endl;
                    new_action_time = now;
                }
            }
        });
}

void CustomActionHandler::send_progress_status(const mavsdk::CustomAction::ActionToExecute& action) {
    while (!_action_stopped.load() && _actions_result.back() != mavsdk::CustomAction::Result::Unknown) {
        mavsdk::CustomAction::ActionToExecute action_exec{};
        action_exec.id = action.id;
        action_exec.progress = _actions_progress.back();
        auto action_result = _actions_result.back();

        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        // Send response with the result and the progress
        _custom_action->respond_custom_action(action_exec, action_result);

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

void CustomActionHandler::process_custom_action(const mavsdk::CustomAction::ActionToExecute& action) {
    std::cout << customActionHandlerOut << " Custom action #" << action.id << " being processed" << std::endl;

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
    std::cout << customActionHandlerOut << " Custom action #" << _actions_metadata.back().id
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

void CustomActionHandler::update_action_progress_from_stage(
    const unsigned& stage_idx, const mavsdk::CustomAction::ActionMetadata& action_metadata) {
    if (!_action_stopped.load()) {
        _actions_progress.back() = (stage_idx + 1.0) / action_metadata.stages.size() * 100.0;

        if (_actions_progress.back() != 100.0) {
            _actions_result.back() = mavsdk::CustomAction::Result::InProgress;
            std::cout << customActionHandlerOut << " Custom action #" << _actions_metadata.back().id
                      << " current progress: " << _actions_progress.back() << "%" << std::endl;
        } else {
            _actions_result.back() = mavsdk::CustomAction::Result::Success;
        }
    }
}

void CustomActionHandler::execute_custom_action(const mavsdk::CustomAction::ActionMetadata& action_metadata) {
    if (!action_metadata.stages.empty()) {
        for (unsigned i = 0; i < action_metadata.stages.size(); i++) {
            mavsdk::CustomAction::Result stage_res = mavsdk::CustomAction::Result::Unknown;

            if (!_action_stopped.load()) {
                std::cout << customActionHandlerOut << " Executing stage " << i << " of action #"
                          << _actions_metadata.back().id << std::endl;

                // Execute the stage and process the result
                std::promise<mavsdk::CustomAction::Result> stage_prom;
                std::future<mavsdk::CustomAction::Result> stage_fut = stage_prom.get_future();
                _custom_action->execute_custom_action_stage_async(
                    action_metadata.stages[i], [&stage_prom, this, i](mavsdk::CustomAction::Result result) {
                        // If one of the stages of the action fails, then the action fails
                        if (result != mavsdk::CustomAction::Result::Success) {
                            std::cout << customActionHandlerOut << " Stage " << i << " of action #"
                                      << _actions_metadata.back().id << " failed!" << std::endl;
                            _actions_result.back() = mavsdk::CustomAction::Result::Error;
                            _action_stopped.store(true, std::memory_order_relaxed);
                        }
                        stage_prom.set_value(result);
                    });
                stage_res = stage_fut.get();

                // TODO: add a way to cancel a script when an action gets canceled

                if (stage_res != mavsdk::CustomAction::Result::Success) {
                    break;
                } else {
                    if (action_metadata.stages[i].state_transition_condition ==
                        mavsdk::CustomAction::Stage::StateTransitionCondition::OnTimeout) {
                        // Advance to next stage after x seconds
                        auto wait_time = action_metadata.stages[i].timeout * 1s;
                        std::unique_lock<std::mutex> lock(cancel_mtx);
                        cancel_signal.wait_for(lock, wait_time, [this]() { return _action_stopped.load(); });
                    } else if (action_metadata.stages[i].state_transition_condition ==
                               mavsdk::CustomAction::Stage::StateTransitionCondition::OnLandingComplete) {
                        // Wait for the vehicle to be landed
                        while (!_action_stopped.load() && (_landed_state == mavsdk::Telemetry::LandedState::Landing ||
                                                           _landed_state != mavsdk::Telemetry::LandedState::OnGround)) {
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        }

                    } else if (action_metadata.stages[i].state_transition_condition ==
                               mavsdk::CustomAction::Stage::StateTransitionCondition::OnTakeoffComplete) {
                        // Wait for the vehicle to finish the takeoff
                        while (!_action_stopped.load() && (_landed_state == mavsdk::Telemetry::LandedState::TakingOff ||
                                                           _landed_state != mavsdk::Telemetry::LandedState::InAir)) {
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        }
                    }

                    update_action_progress_from_stage(i, action_metadata);
                }
            }
        }

    } else if (action_metadata.global_script != "") {
        mavsdk::CustomAction::Result result = mavsdk::CustomAction::Result::Unknown;

        if (!_action_stopped.load()) {
            if (action_metadata.action_complete_condition ==
                mavsdk::CustomAction::ActionMetadata::ActionCompleteCondition::OnResultSuccess) {
                result = _custom_action->execute_custom_action_global_script(action_metadata.global_script);
            } else if (action_metadata.action_complete_condition ==
                       mavsdk::CustomAction::ActionMetadata::ActionCompleteCondition::OnTimeout) {
                std::promise<mavsdk::CustomAction::Result> prom;
                std::future<mavsdk::CustomAction::Result> fut = prom.get_future();
                _custom_action->execute_custom_action_global_script_async(
                    action_metadata.global_script,
                    [&prom](mavsdk::CustomAction::Result script_result) { prom.set_value(script_result); });

                // Consider action complete after x seconds
                auto wait_time = action_metadata.global_timeout * 1s;
                std::unique_lock<std::mutex> lock(cancel_mtx);
                cancel_signal.wait_for(lock, wait_time, [this]() { return _action_stopped.load(); });

                result = fut.get();
            }
        }

        _actions_result.back() = result;

        if (_actions_result.back() == mavsdk::CustomAction::Result::Success) {
            _actions_progress.back() = 100.0;
        }
    }

    // We wait for a bit to make sure that the ACKs are sent
    // to the FMU and don't get lost
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (_actions_result.back() == mavsdk::CustomAction::Result::Timeout) {
        std::cout << customActionHandlerOut << " Custom action #" << _actions_metadata.back().id << " timed-out!"
                  << std::endl;
    } else if (_actions_result.back() == mavsdk::CustomAction::Result::Error) {
        std::cout << customActionHandlerOut << " Custom action #" << _actions_metadata.back().id << " failed!"
                  << std::endl;
    } else if (_actions_result.back() == mavsdk::CustomAction::Result::Success) {
        std::cout << customActionHandlerOut << " Custom action #" << _actions_metadata.back().id << " executed!"
                  << std::endl;
    }

    _action_stopped.store(true, std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    _action_stopped.store(false, std::memory_order_relaxed);

    // clear actions after they are processed
    _actions.clear();
}
