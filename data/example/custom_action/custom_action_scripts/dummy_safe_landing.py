#!/usr/bin/env python3
#############################################################################
#
#   Copyright (c) 2021-2022 Auterion AG. All rights reserved.
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
import asyncio
import sys
from time import sleep
from mavsdk import System
from mavsdk.server_utility import StatusTextType

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
        description="Dummy safe landing script")
    parser.add_argument('-a', '--address', dest='address', action="store",
                        help="mavlink-router UDP IP address", default="",
                        required=False)
    parser.add_argument('-p', '--port', dest='port', action="store",
                        help="mavlink-router UDP port", required=True)
    parser.add_argument('-t', '--timeout', dest='timeout', action="store",
                        help="Amount of time, in seconds, to wait for system connection",
                        default=3.0, required=False, type=float)

    args = parser.parse_args()

    # Init own system (1:MAV_COMP_ID_USER12)
    system = System(sysid=1, compid=36)

    # Wait for a system to be connected for X seconds
    # If timeout, exit with error
    try:
        await asyncio.wait_for(get_connection(system, args.address, args.port), timeout=args.timeout)

        print("\n[Custom Action Script ]  - Safe landing started!\n")
        # Send STATUSTEXT MAVLink message
        await system.server_utility.send_status_text(StatusTextType.NOTICE, 'Safe landing started!')

        sleep(1)

        print(
            "[Custom Action Script ]  - Executing area verification while descending...\n")
        # Send STATUSTEXT MAVLink message
        await system.server_utility.send_status_text(StatusTextType.NOTICE, 'Executing area verification while descending...')

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
