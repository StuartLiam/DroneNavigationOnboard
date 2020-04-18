"""
This script shows a simple scripted flight path using the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        # We take off when the commander is created
        cf = scf.cf
        with MotionCommander(scf) as mc:
            print('Taking off!')
            time.sleep(2)

            roll    = 0.0
            pitch   = 0.0
            yawrate = 90
            thrust  = 0
            #cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 0.4)
            cf.commander.send_hover_setpoint(roll, pitch, yawrate, thrust)

            # We land when the MotionCommander goes out of scope
            print('Landing!')