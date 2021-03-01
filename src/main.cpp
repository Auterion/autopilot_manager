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
 * @file main.cpp
 *
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/signalfd.h>
#include <unistd.h>

#include <mavsdk/mavsdk.h>

#include <AutopilotManager.hpp>
#include "helpers/helpers.hpp"
#include "modules/MissionManager.hpp"

#include <DbusInterface.hpp>

int main(int argc, char* argv[]) {
	uint32_t mavlink_port{14590};
	std::string path_to_apm_config_file{
	    "/shared_container_dir/autopilot_manager.conf"};
	std::string path_to_custom_action_file{
	    "/usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json"};
	// std::string
	// path_to_custom_action_file{"/usr/local/share/autopilot-manager/data/example/custom_action/custom_action.json"};

	parse_argv(argc, argv, mavlink_port, path_to_apm_config_file, path_to_custom_action_file);

	std::cout << mavlink_port;

	// Configure MAVSDK Mission Manager instance
	mavsdk::Mavsdk mavsdk_mission_computer;

	// Change configuration so the instance is treated as a mission computer
	mavsdk::Mavsdk::Configuration config_cc(mavsdk::Mavsdk::Configuration::UsageType::CompanionComputer);
	mavsdk_mission_computer.set_configuration(config_cc);

	GMainLoop* mainloop = g_main_loop_new(NULL, false);
	std::shared_ptr<AutopilotManager> autopilot_manager =
	    std::make_shared<AutopilotManager>(path_to_apm_config_file);

	std::cout << "Autopilot Manager Enabled: " << std::boolalpha << autopilot_manager->autopilotManagerEnabled() << std::endl;

	auto system = std::shared_ptr<mavsdk::System>{nullptr};

	mavsdk::ConnectionResult ret_comp = mavsdk_mission_computer.add_udp_connection(mavlink_port);
	if (ret_comp == mavsdk::ConnectionResult::Success) {
		std::cout << "[AutopilotManagerMain] Waiting to discover vehicle from the mission computer side..."
			  << std::endl;
		std::promise<void> prom;
		std::future<void> fut = prom.get_future();

		mavsdk_mission_computer.subscribe_on_new_system([&prom, &mavsdk_mission_computer, &system]() {
			if (system == nullptr) {
				for (const auto& sys : mavsdk_mission_computer.systems()) {
					if (sys->has_autopilot()) {
						system = sys;
						prom.set_value();
						break;
					}
				}
			}
		});

		fut.wait_for(std::chrono::seconds(10));

		// Start the Mission Manager module
		auto mission_manager = std::make_shared<MissionManager>(system, path_to_custom_action_file);

	} else {
		std::cerr << "[AutopilotManagerMain] Failed to connect to port " << mavlink_port << std::endl;
	}

	g_main_loop_run(mainloop);
	exit(0);
}
