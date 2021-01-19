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
.. automodule:: dummy-winch-control
   :platform: Unix
   :synopsis:
       Dummy winch control script
   :members:

.. codeauthor:: Nuno Marques <nuno@auterion.com>
"""

import argparse

try:
    from pymavlink import mavutil
except:
    print("Failed to import pymavlink.")
    print("You may need to install it with 'pip3 install pymavlink pyserial'")
    print("")
    raise


def main():
    ''' Main funtion '''

    # Parse CLI arguments
    parser = argparse.ArgumentParser(
        description="Dummy winch control script")
    parser.add_argument('-a', '--address', dest='address', action="store",
                        help="mavlink-router UDP IP address", required=True)
    parser.add_argument('-p', '--port', dest='port', action="store",
                        help="mavlink-router UDP port", required=True)
    parser.add_argument('--descend', dest='activation', action="store_true",
                        help="Descend winch")
    parser.add_argument('--raise', dest='activation', action="store_false",
                        help="Raise winch")
    parser.set_defaults(activation=False)

    args = parser.parse_args()

    # Create a MAVLink connection through a specific UDP port
    gcs = mavutil.mavlink_connection(
        'udp:' + args.address + ':' + args.port, source_system=1)
    gcs.wait_heartbeat()

    if args.activation:
        print("\n - Descending payload...\n");
        # Send STATUSTEXT MAVLink message
        gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, b"Descending payload...")
    else:
        print("\n - Descending payload...\n");
        # Send STATUSTEXT MAVLink message
        gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, b"Raising winch...")

    gcs.close()


if __name__ == '__main__':
    main()
