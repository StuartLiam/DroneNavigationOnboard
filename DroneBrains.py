# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
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
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import Virtual as vr
import GraphDomain as gd
import LineHelp as lh
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import matplotlib.patches as patches
import numpy as np

import logging
import sys
import time
import math

# URI to the Crazyflie to connect to
URI = 'radio://0/80/2M/E7E7E7E7E7'

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


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)




def slightly_more_complex_usage():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
                default_velocity=0.3,
                default_height=0.5,
                controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
            # Go to a coordinate
            pc.go_to(1.0, 1.0, 1.0)

            # Move relative to the current position
            pc.right(1.0)

            # Go to a coordinate and use default height
            pc.go_to(0.0, 0.0)

            # Go slowly to a coordinate
            pc.go_to(1.0, 1.0, velocity=0.2)

            # Set new default velocity and height
            pc.set_default_velocity(0.3)
            pc.set_default_height(1.0)
            pc.go_to(0.0, 0.0)


def simple_sequence():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(scf) as pc:
            pc.forward(1.0)
            pc.left(1.0)
            pc.back(1.0)
            pc.go_to(0.0, 0.0, 1.0)

def goTo(drone,node):
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                x=drone.x, y=drone.y, z=0.4,
                default_velocity=0.2,
                default_height=0.4,
                controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:

            # Go to a coordinate and use default height
            #pc.go_to(0.0, 0.0)

            # Go slowly to a coordinate
            pc.go_to(node.x, node.y, velocity=0.2)

def blocked():
     # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:

                VELOCITY = 0.5
                velocity_x = 0.0
                velocity_y = 0.0

                if is_close(multiranger.front):
                    motion_commander.start_linear_motion(
                        velocity_x, velocity_y, 0)
                    time.sleep(0.1)
                

    def scan(drone):
        print("hold")
        with SyncCrazyflie(URI) as scf:
        # We take off when the commander is created
            with MotionCommander(scf) as mc:
                with Multiranger(scf) as multiranger:
            
                    initialYaw = drone.yaw_deg
                
                    countback = 0
                    leftNode = 0
                    rightNode = 0
                    while(multiranger.front < 1.0):
                        
                        if(drone.yaw_deg > 180):

                            if(drone.yaw_deg > 270):
                                print("q4")                            
                                off_x = math.sin(drone.yaw_deg-270)*multiranger.front
                                off_y = math.cos(drone.yaw_deg-270)*multiranger.front
                                leftNode = gd.Node(drone.x+off_x,drone.y-off_y)
                                
                            else:
                                print("q3")
                                off_x = math.cos(drone.yaw_deg-180)*multiranger.front
                                off_y = math.sin(drone.yaw_deg-180)*multiranger.front
                                leftNode = gd.Node(drone.x-off_x,drone.y-off_y)
                        else:
                            if(drone.yaw_deg > 90):
                                print("q2")                            
                                off_x = math.sin(drone.yaw_deg-90)*multiranger.front
                                off_y = math.cos(drone.yaw_deg-90)*multiranger.front
                                leftNode = gd.Node(drone.x-off_x,drone.y+off_y)
                                
                            else:
                                print("q1")
                                off_x = math.cos(drone.yaw_deg)*multiranger.front
                                off_y = math.sin(drone.yaw_deg)*multiranger.front
                                leftNode = gd.Node(drone.x-off_x,drone.y+off_y)

                        mc.circle_left(5)
                        drone.yaw_deg +=5

                        if drone.yaw_deg > 360:
                            drone.yaw_deg -=360

                    mc.circle_right(5)

                    while(multiranger.front < 1.0):
                        
                        if(drone.yaw_deg > 180):

                            if(drone.yaw_deg > 270):
                                print("q4")                            
                                off_x = math.sin(drone.yaw_deg-270)*multiranger.front
                                off_y = math.cos(drone.yaw_deg-270)*multiranger.front
                                rightNode = gd.Node(drone.x+off_x,drone.y-off_y)
                                
                            else:
                                print("q3")
                                off_x = math.cos(drone.yaw_deg-180)*multiranger.front
                                off_y = math.sin(drone.yaw_deg-180)*multiranger.front
                                rightNode = gd.Node(drone.x-off_x,drone.y-off_y)
                        else:
                            if(drone.yaw_deg > 90):
                                print("q2")                            
                                off_x = math.sin(drone.yaw_deg-90)*multiranger.front
                                off_y = math.cos(drone.yaw_deg-90)*multiranger.front
                                rightNode = gd.Node(drone.x-off_x,drone.y+off_y)
                                
                            else:
                                print("q1")
                                off_x = math.cos(drone.yaw_deg)*multiranger.front
                                off_y = math.sin(drone.yaw_deg)*multiranger.front
                                rightNode = gd.Node(drone.x-off_x,drone.y+off_y)

                        mc.circle_right(5)
                        drone.yaw_deg -=5
                        if(drone.yaw_deg < 0):
                            drone.yaw_deg +=360

                

                # We land when the MotionCommander goes out of scope
        return [leftNode,rightNode]

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    simple_sequence()
    # slightly_more_complex_usage()
