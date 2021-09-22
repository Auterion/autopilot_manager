#!/usr/bin/env python3
#############################################################################
#
#   Copyright (c) 2021 Auterion AG. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#############################################################################


"""
.. automodule:: dummy-safe-landing
   :platform: Unix
   :synopsis:
       Dummy safe landing script
   :members:

.. codeauthor:: Nuno Marques <nuno@auterion.com>
"""

import argparse
import time

try:
    from pymavlink import mavutil
except:
    print("Failed to import pymavlink.")
    print("You may need to install it with 'pip3 install pymavlink pyserial'")
    print("")
    raise


landing_state = mavutil.mavlink.enums['MAV_LANDED_STATE'][0]


def handle_extended_sys_state(msg) -> None:
    """Handle EXTENDED_SYS_STATE MAVLink message."""
    global landing_state
    landing_state = msg.landed_state


def main() -> None:
    """Main funtion."""
    # Parse CLI arguments
    parser = argparse.ArgumentParser(
        description="Dummy winch control script")
    parser.add_argument('-a', '--address', dest='address', action="store",
                        help="mavlink-router UDP IP address", required=True)
    parser.add_argument('-p', '--port', dest='port', action="store",
                        help="mavlink-router UDP port", required=True)

    args = parser.parse_args()

    # Create a MAVLink connection through a specific UDP port
    gcs = mavutil.mavlink_connection(
        'udp:' + args.address + ':' + args.port, source_system=1)
    gcs.wait_heartbeat()

    print("\n - Safe landing started!\n")
    # Send STATUSTEXT MAVLink message
    gcs.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_NOTICE, b"Safe landing started!")

    time.sleep(1)

    print(" - Executing area verification while descending...\n")
    # Send STATUSTEXT MAVLink message
    gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                            b"Executing area verification while descending...")

    while landing_state != 1:  # MAV_LANDED_STATE_ON_GROUND
        msg = gcs.recv_match(blocking=True)
        msg_type = msg.get_type()
        if msg_type == "EXTENDED_SYS_STATE":
            handle_extended_sys_state(msg)

    print(" - Safe land executed.\n")
    # Send STATUSTEXT MAVLink message
    gcs.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_NOTICE, b"Safe land executed!")

    gcs.close()


if __name__ == '__main__':
    main()
