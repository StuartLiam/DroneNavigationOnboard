#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Python Library
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Example scipts that allows a user to "push" the Crazyflie 2.0 around
using your hands while it's hovering.

This examples uses the Flow and Multi-ranger decks to measure distances
in all directions and tries to keep away from anything that comes closer
than 0.2m by setting a velocity in the opposite direction.

The demo is ended by either pressing Ctrl-C or by holding your hand above the
Crazyflie.

For the example to run the following hardware is needed:
 * Crazyflie 2.0
 * Crazyradio PA
 * Flow deck
 * Multiranger deck
"""
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

import Virtual as vr
import GraphDomain as gd

URI = 'radio://0/80/250K'






room = vr.World(0,0,30,30,[gd.Node(20,20)])
droneActor = room.drone

HORIZONTAL = 1
VERITCAL = 2

world.drone.x = 5
world.drone.y = 5



if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True

                while keep_flying:

                    while(room.goals!=None):
                        VELOCITY = 0.5
                        velocity_x = 0.0
                        velocity_y = 0.0

                        if is_close(multiranger.front):
                            velocity_x -= VELOCITY
                            room.split(droneActor.x, droneActor.y, HORIZONTAL, False)

                        if is_close(multiranger.back):
                            velocity_x += VELOCITY
                            room.split(droneActor.x, droneActor.y, HORIZONTAL, False)

                        if is_close(multiranger.left):
                            velocity_y -= VELOCITY
                            room.split(droneActor.x, droneActor.y, VERITCAL, False)

                        if is_close(multiranger.right):
                            velocity_y += VELOCITY
                            room.split(droneActor.x, droneActor.y, VERITCAL, False)

                        if is_close(multiranger.up):
                            keep_flying = False

                        motion_commander.start_linear_motion(
                            velocity_x, velocity_y, 0)


                        for i in room.blocks:
                            droneBlock = i if (i.inside(room.drone.x,room.drone.y)) else droneBlock
                            goalBlock = i if (i.inside(room.currentGoal.x,room.currentGoal.y)) else goalBlock
                        
                        if(droneBlock == goalBlock): #same block
                            print("are in same block")
                        
                        else:
                            print("in different nodes")
                            

                        #check useful nodes
                        #Update the nodes distance to the goal
                        #if node can get closer goto your blocks node and transfer nodes
                        # also keep track of used nodes

                        #if not in your block and are in best block
                        # do a perameter check

                        # if last check was not closed and now closed split
                        # if last check was closed and now not closed split

                        #Parameter check !!!!!! on bad split

                        time.sleep(0.1)

            print('Demo terminated!')
