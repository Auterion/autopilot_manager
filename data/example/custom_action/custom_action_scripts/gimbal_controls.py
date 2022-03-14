#!/usr/bin/env python3
#############################################################################
#
#   Copyright (c) 2022 Auterion AG. All rights reserved.
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
.. automodule:: gimbal-controls
   :platform: Unix
   :synopsis:
       Example script for controlling the gimbal
   :members:

.. codeauthor:: Yannick Fuhrer <yannick@auterion.com>
"""

import argparse
import asyncio
import sys
from time import sleep
from mavsdk import System
from mavsdk.gimbal import ControlMode

# Global return exit value
ret = 0


async def get_connection(system, ip_address, udp_port):
    """Get connection."""
    # Create a MAVLink connection to a system through a specific IP address
    # and UDP port
    await system.connect(system_address="udp://" + ip_address + ":" + udp_port)

    print("[Custom Action Script ] Waiting for system to connect...")
    async for state in system.core.connection_state():
        if state.is_connected:
            print(f"[Custom Action Script ] System discovered!")
            break


async def run() -> None:
    """Main funtion."""
    # Parse CLI arguments
    parser = argparse.ArgumentParser(
        description="Example script for controlling a gimbal")
    parser.add_argument('-a', '--address', dest='address', action="store",
                        help="mavlink-router UDP IP address", default="",
                        required=False)
    parser.add_argument('-p', '--port', dest='port', action="store",
                        help="mavlink-router UDP port", required=True)
    parser.add_argument('-t', '--timeout', dest='timeout', action="store",
                        help="Amount of time, in seconds, to wait for system connection",
                        default=3.0, required=False, type=float)
    parser.add_argument(
        '--right', help="Change gimbal heading 90 degrees right", action="store_true")
    parser.add_argument(
        '--left', help="Change gimbal heading 90 degrees left", action="store_true")
    parser.add_argument(
        '--down', help="Change gimbal pitch 90 degrees down", action="store_true")
    args = parser.parse_args()

    # Init own system (1:MAV_COMP_ID_USER14)
    system = System(sysid=1, compid=38)

    # Wait for a system to be connected for X seconds
    # If timeout, exit with error
    try:
        await asyncio.wait_for(get_connection(system, args.address, args.port), timeout=args.timeout)

        if args.right:
            print(
                "\n[Custom Action Script ]  - Gimbal looking right for 3 seconds...\n")
            # take control over gimbal
            await system.gimbal.take_control(ControlMode.PRIMARY)
            # control gimbal
            await system.gimbal.set_pitch_and_yaw(0, 90)
            sleep(3)
            await system.gimbal.set_pitch_and_yaw(0, 0)
            # release control
            await system.gimbal.release_control()

        elif args.left:
            print(
                "\n[Custom Action Script ]  - Gimbal looking left for 3 seconds...\n")
            # take control over gimbal
            await system.gimbal.take_control(ControlMode.PRIMARY)
            # control gimbal
            await system.gimbal.set_pitch_and_yaw(0, -90)
            sleep(3)
            await system.gimbal.set_pitch_and_yaw(0, 0)
            # release control
            await system.gimbal.release_control()

        elif args.down:
            print(
                "\n[Custom Action Script ]  - Gimbal looking down for 3 seconds...\n")
            # take control over gimbal
            await system.gimbal.take_control(ControlMode.PRIMARY)
            # control gimbal
            await system.gimbal.set_pitch_and_yaw(-90, 0)
            sleep(3)
            await system.gimbal.set_pitch_and_yaw(0, 0)
            # release control
            await system.gimbal.release_control()

    except asyncio.TimeoutError:
        global ret
        ret = 1
        print(
            '[Custom Action Script ] Failed to connect to system after {} seconds. Action failed!'.format(args.timeout))


if __name__ == '__main__':
    # Start the event loop
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run())
    finally:
        sys.exit(ret)
