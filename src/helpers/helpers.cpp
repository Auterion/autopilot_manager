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

#include "helpers.hpp"

void help_argv_description(const char* pgm) {
	std::cout << pgm
		  << " [OPTIONS...]\n\n"
		     "  -a --file-custom-action-config	Absolute path to configuration file of the custom actions.\n"
		     "\t\t\t\t\tDefault: /usr/src/app/autopilot-manager/data/example/custom_action/custom_action.json\n"
		     "  -c --file-autopilot-manager-config	Absolute path to configuration file of the overall "
		     "autopilot manager service.\n"
		     "\t\t\t\t\tDefault: /shared_container_dir/autopilot_manager.conf\n"
		     "  -m --mavlink-port			MAVLink port to connect the Autopilot "
		     "Manager MAVSDK instance\n\t\t\t\t\tthrough UDP. Default: 14570\n"
		     "  -h --help				Print this message\n";
}

void parse_argv(int argc, char* const argv[], uint32_t& mavlink_port, std::string& path_to_apm_config_file,
		std::string& path_to_custom_action_config_file) {
	static const struct option options[] = {{"file-custom-action-config", required_argument, nullptr, 'a'},
						{"file-autopilot-manager-config", required_argument, nullptr, 'c'},
						{"mavlink-port", required_argument, nullptr, 'm'},
						{"help", no_argument, nullptr, 'h'}};

	int c;
	bool invalid_argument = false;

	while ((c = getopt_long(argc, argv, "a:c:hm:", options, nullptr)) >= 0) {
		switch (c) {
			case 'h':
				help_argv_description(argv[0]);
				exit(0);
			case 'a':
				path_to_custom_action_config_file = std::string(optarg);
				break;
			case 'c':
				path_to_apm_config_file = std::string(optarg);
				break;
			case 'm':
				if (!atoi(optarg)) {
					invalid_argument = true;
				} else {
					mavlink_port = atoi(optarg);
				}
				break;
			case '?':
			default:
				help_argv_description(argv[0]);
				exit(-1);
		}
	}
	/* positional arguments */
	if (optind != argc || invalid_argument) {
		std::cerr << "Invalid argument" << std::endl;
		help_argv_description(argv[0]);
		exit(-1);
	}
}
