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
.. automodule:: dummy-cargo-compartment
   :platform: Unix
   :synopsis:
       Dummy cargo compartment control script
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
        description="Dummy cargo compartment control script")
    parser.add_argument('-a', '--address', dest='address', action="store",
                        help="mavlink-router UDP IP address", default="",
                        required=False)
    parser.add_argument('-p', '--port', dest='port', action="store",
                        help="mavlink-router UDP port", required=True)
    parser.add_argument('-t', '--timeout', dest='timeout', action="store",
                        help="Amount of time, in seconds, to wait for system connection",
                        default=3.0, required=False, type=float)
    parser.add_argument('--open', dest='activation', action="store_true",
                        help="Open cargo compartment latch")
    parser.add_argument('--close', dest='activation', action="store_false",
                        help="Close cargo compartment latch")
    parser.set_defaults(activation=False)

    args = parser.parse_args()

    # Init own system (1:MAV_COMP_ID_USER11)
    system = System(sysid=1, compid=35)

    # Wait for a system to be connected for 3 seconds
    # If timeout, exit with error
    try:
        await asyncio.wait_for(get_connection(system, args.address, args.port), timeout=args.timeout)

        if args.activation:
            print("\n[Custom Action Script ]  - Opening cargo compartment latch...\n")
            # Send STATUSTEXT MAVLink message
            await system.server_utility.send_status_text(StatusTextType.NOTICE, 'Opening cargo compartment latch...')
            sleep(3)
        else:
            print("\n[Custom Action Script ]  - Closing cargo compartment latch...\n")
            await system.server_utility.send_status_text(StatusTextType.NOTICE, 'Closing cargo compartment latch...')
            sleep(3)

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
