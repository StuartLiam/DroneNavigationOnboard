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
#  Crazyflie Nano Quadcopter Client
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
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

The MotionCommander uses velocity setpoints.

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

URI = 'radio://0/80/2M'

world = vr.World(0,0,6,6,[gd.Node(2,2)])
world.drone.x = 3
world.drone.y = 3

def is_close(range):
    MIN_DISTANCE = 0.4  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def is_far(range):
    MIN_DISTANCE = 0.8  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def detect_to_Node(xStart,yStart,angle,distance):
    return  gd.Node(distance*math.cos(angle)+xStart, distance*math.sin(angle)+yStart)


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    #log_config.add_variable('kalman.varPYaw', 'float')

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

def scan():
    #with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
         with MotionCommander(scf) as mc:
            mc.start_circle_left(360)
            time.sleep(1)
            
            mc.stop

def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    
    world.drone.x = x
    world.drone.y = y

    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='kalman Variance', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    #log_conf.add_variable('kalman.yaw','float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def start_yaw_printing(scf):
    log_conf = LogConfig(name='stateEstimate', period_in_ms=500)
    log_conf.add_variable('stateEstimate.yaw','float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(yaw_callback)
    log_conf.start()

def yaw_callback(tilestamp,data,logconf):
    yaw = data['stateEstimate.yaw']
    world.drone.yaw_deg = yaw
    print(world.drone.yaw_deg)

def scan(scf):
    with MotionCommander(scf) as mc:
        with Multiranger(scf) as multiranger:

            mc.start_turn_left(10)
            while(is_far(multiranger.front)):
                do = 0
            print("found left\n\n\n\n")

            leftNode = detect_to_Node(world.drone.x,world.drone.y,world.drone.yaw_deg,multiranger.front)
            mc.start_turn_right(10)
            time.sleep(1.5)
            while(is_far(multiranger.front)):
                do = 0
            rightNode = detect_to_Node(world.drone.x,world.drone.y,world.drone.yaw_deg,multiranger.front)
            print("found right\n\n\n\n")
            mc.stop
    return [leftNode,rightNode]

def careful_Forward(scf):
    with MotionCommander(scf) as mc:
        with Multiranger(scf) as multiranger:


            # There is a set of functions that move a specific distance
            # We can move in all directions
            #mc.VELOCITY = 0.2

            mc.start_forward(velocity=0.3)


            while(not is_close(multiranger.front) ):
                do = 0
            print("found\n\n\n\n")
            mc.stop
            return scan(scf)

def droneNear(Node):
    print("hold")

def droneAt(Node):
    print("hold")

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        initial_x = 3.0
        initial_y = 3.0
        initial_z = 0.0
        initial_yaw = 90  # In degrees
        cf = scf.cf
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        start_position_printing(scf)
        start_yaw_printing(scf)
      
        careful_Forward(scf)




    world.graph.updateWeights(world.blocks[0].node)
    world.adjustForSize()
    print("testing for bad paths\n")
    while(world.checkPaths()):
        world.updateGraph()
    print("final results\n")

    world.print()

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, aspect='equal')
    ax2.margins(x=7,y=7)
    for i in world.blocks:
        ax2.add_patch(
            patches.Rectangle(
                (i.x, i.y),
                i.width,
                i.height,
                fill=False if (i.flyable) else True      # remove background
            ) ) 
    fig2.savefig('rect2.png', dpi=90, bbox_inches='tight')

    plt.show()