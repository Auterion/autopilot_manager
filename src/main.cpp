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
 * @brief Autopilot Manager main
 * @file main.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <AutopilotManager.hpp>
#include <DbusInterface.hpp>
#include <helpers.hpp>
#include <rclcpp/rclcpp.hpp>

constexpr uint32_t mavlink_router_port = 14590;

auto main(int argc, char* argv[]) -> int {
    uint32_t mavlink_port{mavlink_router_port};
    std::string path_to_apm_config_file{"/shared_container_dir/autopilot-manager/data/config/autopilot_manager.conf"};
    std::string path_to_custom_action_file{
        "/shared_container_dir/autopilot-manager/data/custom_action/custom_action.json"};

    // Initialize communications via the rmw implementation and set up a global signal handler.
    rclcpp::init(argc, argv, rclcpp::InitOptions());

    // Uninstall the global signal handler for rclcpp
    rclcpp::uninstall_signal_handlers();

    // Init main event loop for GLib/DBUS
    GMainLoop* mainloop = g_main_loop_new(nullptr, static_cast<gboolean>(false));

    // Extract paths to config files
    // WARNING: This alters the ordering of argv
    parse_argv(argc, argv, mavlink_port, path_to_apm_config_file, path_to_custom_action_file);

    auto autopilot_manager = std::make_shared<AutopilotManager>(std::to_string(mavlink_port), path_to_apm_config_file,
                                                                path_to_custom_action_file);

    // Register autopilot_manager dbus requests
    DBusInterface dbus([autopilot_manager](DBusMessage* request) { return autopilot_manager->HandleRequest(request); });

    g_main_loop_run(mainloop);

    rclcpp::shutdown();

    return 0;
}
