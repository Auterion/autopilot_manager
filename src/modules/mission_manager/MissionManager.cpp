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
 * @file MissionManager.cpp
 *
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <atomic>
#include <future>
#include <iostream>
#include <string>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/signalfd.h>
#include <unistd.h>

#include <MissionManager.hpp>

using namespace std::chrono_literals;

static std::atomic<bool> int_signal{false};

MissionManager::MissionManager(std::shared_ptr<mavsdk::System> system, const std::string &path_to_custom_action_file)
    : _config_update_callback([]() { return MissionManagerConfiguration{}; }),
      _mavsdk_system{system},
      _path_to_custom_action_file{path_to_custom_action_file},
      _mission_manager_config{} {}

MissionManager::~MissionManager() {
	deinit();
}

void MissionManager::init() {
	std::cout << "[Mission Manager] Started!" << std::endl;
	_custom_action_handler = std::make_shared<CustomActionHandler>(_mavsdk_system, _path_to_custom_action_file);

	run();
}

void MissionManager::deinit() {
	_custom_action_handler.reset();
}

void MissionManager::run() {
	auto decision_maker_th = std::thread(&MissionManager::decision_maker_run, this);

	// Start custom action handler
	if (_custom_action_handler->start()) {
		_custom_action_handler->run();
	}

	decision_maker_th.join();
}

void MissionManager::decision_maker_run() {
	while (!int_signal) {
		// Update configuration at each iteration
		_mission_manager_config = _config_update_callback();

		// std::cout << _mission_manager_config.decision_maker_input_type << std::endl;

		if (_mission_manager_config.decision_maker_input_type == "SIMPLE_COLLISION_AVOIDANCE") {
			if (_mission_manager_config.simple_collision_avoid_enabled) {
				// std::cout << "Collision avoidance enabled" << std::endl;

				if (_mavsdk_system->is_connected() && _mavsdk_system->has_autopilot()) {
					// Actions are processed and executed in the Mission Manager
					_action = std::make_shared<mavsdk::Action>(_mavsdk_system);
				}
			}
		}

		// Decision maker runs at 10hz
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

CustomActionHandler::CustomActionHandler(std::shared_ptr<mavsdk::System> system,
					 const std::string &path_to_custom_action_file)
    : _mavsdk_system{system}, _path_to_custom_action_file{path_to_custom_action_file} {}

bool CustomActionHandler::start() {
	if (_mavsdk_system->is_connected() && _mavsdk_system->has_autopilot()) {
		// Telemetry checks are fundamental for proper execution
		_telemetry = std::make_shared<mavsdk::Telemetry>(_mavsdk_system);

		// Custom actions are processed and executed in the Mission Manager
		_custom_action = std::make_shared<mavsdk::CustomAction>(_mavsdk_system);

		return true;
	} else {
		return false;
	}
}

void CustomActionHandler::run() {
	// while (!_telemetry->health_all_ok()) {
	// 	std::cout << "[Mission Manager] Waiting for system to be ready" << std::endl;
	// 	std::this_thread::sleep_for(std::chrono::seconds(1));
	// }

	std::cout << "[Mission Manager] System ready! Waiting for custom actions to process..." << std::endl;

	// Subscribe to the cancelation message
	_custom_action->subscribe_custom_action_cancellation(
	    [this](bool canceled) { _action_stopped.store(canceled, std::memory_order_relaxed); });

	auto new_actions_check_th = std::thread(&CustomActionHandler::new_action_check, this);

	auto new_action_time = std::chrono::system_clock::now() + std::chrono::minutes(1);  // dummy init
	auto start_time = new_action_time;

	// Get the custom action to process
	_custom_action->subscribe_custom_action(
	    [this, &start_time, &new_action_time](mavsdk::CustomAction::ActionToExecute action_to_exec) {
		    if (_actions.empty() || (!_actions.empty() && _actions.back().id != action_to_exec.id)) {
			    auto now = std::chrono::system_clock::now();
			    // This is a safeguard and a workaround in case the FMU sends consecutive MAV_CMDs because
			    // he didn't get an ACK
			    if (new_action_time == start_time ||
				std::chrono::duration_cast<std::chrono::milliseconds>(now - new_action_time).count() >=
				    1500) {
				    _actions.push_back(action_to_exec);
				    _new_action.store(true, std::memory_order_relaxed);
				    std::cout << "[Mission Manager] New action received with ID " << action_to_exec.id
					      << std::endl;
				    new_action_time = now;
			    }
		    }
	    });

	while (true) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	int_signal.store(true, std::memory_order_relaxed);
	new_actions_check_th.join();
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
		_custom_action->respond_custom_action_async(action_exec, action_result,
							    [&prom](mavsdk::CustomAction::Result result) {
								    // if (result != CustomAction::Result::Success) {
								    //
								    // }
								    prom.set_value();
							    });

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
	std::cout << "[Mission Manager] Custom action #" << action.id << " being processed" << std::endl;

	// Get the custom action metadata
	std::promise<mavsdk::CustomAction::ActionMetadata> prom;
	std::future<mavsdk::CustomAction::ActionMetadata> fut = prom.get_future();
	_custom_action->custom_action_metadata_async(
	    action, _path_to_custom_action_file,
	    [&prom](mavsdk::CustomAction::Result result, mavsdk::CustomAction::ActionMetadata action_metadata) {
		    prom.set_value(action_metadata);
		    // if (result != mavsdk::CustomAction::Result::Success) {
		    //
		    // }
	    });

	_actions_metadata.push_back(fut.get());

	// Start
	_actions_result.push_back(mavsdk::CustomAction::Result::InProgress);
	_actions_progress.push_back(0.0);
	std::cout << "[Mission Manager] Custom action #" << _actions_metadata.back().id
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

void CustomActionHandler::execute_custom_action(mavsdk::CustomAction::ActionMetadata action_metadata) {
	if (!action_metadata.stages.empty()) {
		for (unsigned i = 0; i < action_metadata.stages.size(); i++) {
			if (!_action_stopped.load()) {
				mavsdk::CustomAction::Result stage_res =
				    _custom_action->execute_custom_action_stage(action_metadata.stages[i]);
				// if (stage_res != mavsdk::CustomAction::Result::Success) {
				//
				// }
			}

			auto wait_time = action_metadata.stages[i].timestamp_stop * 1s;
			std::unique_lock<std::mutex> lock(cancel_mtx);
			cancel_signal.wait_for(lock, wait_time, [this]() { return _action_stopped.load(); });

			if (!_action_stopped.load()) {
				_actions_progress.back() = (i + 1.0) / action_metadata.stages.size() * 100.0;

				if (_actions_progress.back() != 100.0) {
					_actions_result.back() = mavsdk::CustomAction::Result::InProgress;
					std::cout << "[Mission Manager] Custom action #" << _actions_metadata.back().id
						  << " current progress: " << _actions_progress.back() << "%"
						  << std::endl;
				} else {
					_actions_result.back() = mavsdk::CustomAction::Result::Success;
				}
			}
		}

	} else if (action_metadata.global_script != "") {
		if (!_action_stopped.load()) {
			std::promise<mavsdk::CustomAction::Result> prom;
			std::future<mavsdk::CustomAction::Result> fut = prom.get_future();
			_custom_action->execute_custom_action_global_script_async(
			    action_metadata.global_script,
			    [&prom](mavsdk::CustomAction::Result script_result) { prom.set_value(script_result); });

			mavsdk::CustomAction::Result result = mavsdk::CustomAction::Result::Unknown;
			if (!std::isnan(action_metadata.global_timeout)) {
				std::chrono::seconds timeout(static_cast<long int>(action_metadata.global_timeout));
				while (fut.wait_for(timeout) != std::future_status::ready) {
				};
			}

			result = fut.get();
			// if (result != mavsdk::CustomAction::Result::Success) {
			//
			// }

			_actions_result.back() = result;

			if (result == mavsdk::CustomAction::Result::Success) {
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
		std::cout << "[Mission Manager] Custom action #" << _actions_metadata.back().id << " canceled!"
			  << std::endl;
		_action_stopped.store(false, std::memory_order_relaxed);
	} else if (!_action_stopped.load() && _actions_progress.back() == 100) {
		std::cout << "[Mission Manager] Custom action #" << _actions_metadata.back().id << " executed!"
			  << std::endl;
	}

	// clear actions after they are processed
	_actions.clear();
}
