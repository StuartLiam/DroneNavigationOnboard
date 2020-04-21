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

import VirtualFix as vr
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

world = vr.World(0,0,2,2,[gd.Node(1,1.9)])
world.drone.x = 1
world.drone.y = 1

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

def detect_to_Node(xStart,yStart,a,d):
    print("angle: {}".format(a))
    print("dist: {}".format(d))
    dx = d*math.cos(math.radians(a))
    dy = d*math.sin(math.radians(a))
    print (dx)
    print (dy)
    return gd.Node(dx+xStart,dy+yStart,)


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

    yaw_radian = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radian)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    
    world.drone.x = x
    world.drone.y = y

   # print('pos: ({}, {}, {})'.format(x, y, z))


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
   # print(world.drone.yaw_deg)

def dronePointAngle(Node):
    if(int(world.drone.x)==Node.x):#same x
        if(Node.y > world.drone.y):
            return 90
        else:
            return 180
    if(int(world.drone.y)==Node.y):
        if(Node.x > world.drone.x):
            return 90
        else:
            return 180

    return math.degrees(math.atan((Node.y-world.drone.y)/(Node.x-world.drone.y)))
    

def atNode(node):
    return abs(world.drone.x - node.x) < 0.2 and abs(world.drone.y - node.y) < 0.2



if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        #world.getDroneBlock().node.print()
        initial_x = 1
        initial_y = 1
        initial_z = 0.0
        initial_yaw = 90  # In degrees
        cf = scf.cf
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        start_position_printing(scf)
        start_yaw_printing(scf)
        with MotionCommander(scf) as mc:
            with Multiranger(scf) as multiranger:

                print(world.drone.x)
                print(world.drone.y)
                world.currentGoal.print()
                turnAngle = dronePointAngle(world.currentGoal) - world.drone.yaw_deg
                print(turnAngle)
                # There is a set of functions that move a specific distance
                # We can move in all directions
                #mc.VELOCITY = 0.2

                time.sleep(2)


                while (not atNode(world.currentGoal) and not is_close(multiranger.up)):

                    #world.getDroneBlock().node.print()
                   # world.getGoalBlock().node.print()

                    if(world.getDroneBlock() is world.getGoalBlock()):

                        print("its same block so to towards")

                        turnAngle = dronePointAngle(world.currentGoal) - world.drone.yaw_deg
                        print("Now turning: {} degrees".format(turnAngle))
                        if(turnAngle < 0):
                            mc.turn_right(abs(turnAngle))
                        else:
                            mc.turn_left(turnAngle)

                        mc.start_forward(velocity=0.2)
                        while(not is_close(multiranger.front) and not atNode(world.currentGoal)): #and not at goal
                            do = 0
                            
                        if(not atNode(world.currentGoal)):
                            print("found while headning to goal\n\n\n\n")


                            mc.start_turn_left(20)
                            while(is_far(multiranger.front)):
                                do = 0
                            print("found left\n\n\n\n")
                            mc.start_turn_right(20)
                            time.sleep(2)
                            xD = world.drone.x
                            yD = world.drone.y
                            yawD = world.drone.yaw_deg
                            dist = multiranger.front
                            print("Drone X: {}, Drone Y: {}, Angle:{}, Distance: {}".format(xD,yD,yawD,dist))
                            
                            leftNode = detect_to_Node(xD,yD,yawD,dist)

                            while(is_far(multiranger.front)):
                                do = 0

                            print("found right\n\n\n\n")
                            mc.start_turn_left(20)
                            time.sleep(1)
                            xD = world.drone.x
                            yD = world.drone.y
                            yawD = world.drone.yaw_deg
                            dist = multiranger.front
                            rightNode = detect_to_Node(xD,yD,yawD,dist)
                            print("Drone X: {}, Drone Y: {}, Angle:{}, Distance: {}".format(xD,yD,yawD,dist))
                            mc.stop

                            world.createWallBlock(leftNode,rightNode)

                        while(world.checkPaths()):
                            world.updateGraph()
                        world.graph.updateWeights(world.currentGoal)

                    else: #deal with different blocks
                        
                        if(atNode(world.getDroneBlock().node)):
                            print("im at the centerpoint, next node dawg")
                            world.updateGraph()
                            while(world.checkPaths()):
                                world.updateGraph()
                            world.graph.updateWeights(world.currentGoal)
                            world.print()
                            nextNode = world.bestNode(world.getDroneBlock().node)

                            turnAngle = dronePointAngle(nextNode) - world.drone.yaw_deg
                            if(turnAngle < 0):
                                mc.turn_right(abs(turnAngle))
                            else:
                                mc.turn_left(turnAngle)

                            mc.start_forward(velocity=0.2)
                            while(not is_close(multiranger.front) and not atNode(nextNode)):
                                do = 0
                            if(not atNode(nextNode)):
                                print("found\n\n\n\n")


                                mc.start_turn_left(20)
                                while(is_far(multiranger.front)):
                                    do = 0
                                print("found left\n\n\n\n")
                                mc.start_turn_right(20)
                                time.sleep(2)
                                xD = world.drone.x
                                yD = world.drone.y
                                yawD = world.drone.yaw_deg
                                dist = multiranger.front
                                print("Drone X: {}, Drone Y: {}, Angle:{}, Distance: {}".format(xD,yD,yawD,dist))
                                
                                leftNode = detect_to_Node(xD,yD,yawD,dist)

                                while(is_far(multiranger.front)):
                                    do = 0

                                print("found right\n\n\n\n")
                                mc.start_turn_left(20)
                                time.sleep(1)
                                xD = world.drone.x
                                yD = world.drone.y
                                yawD = world.drone.yaw_deg
                                dist = multiranger.front
                                rightNode = detect_to_Node(xD,yD,yawD,dist)
                                print("Drone X: {}, Drone Y: {}, Angle:{}, Distance: {}".format(xD,yD,yawD,dist))
                                mc.stop
                                world.createWallBlock(leftNode,rightNode)
                            while(world.checkPaths()):
                                world.updateGraph()
                            world.graph.updateWeights(world.currentGoal)

                        else:
                            print("different block so going to node")
                            turnAngle = dronePointAngle(world.getDroneBlock().node) - world.drone.yaw_deg
                            if(turnAngle < 0):
                                mc.turn_right(abs(turnAngle))
                            else:
                                mc.turn_left(turnAngle)

                            mc.start_forward(velocity=0.2)
                            while(not is_close(multiranger.front) and not atNode(world.getDroneBlock().node)):
                                do = 0
                            if(not atNode(world.getDroneBlock().node)):
                                print("found while going to node\n\n\n\n")


                                mc.start_turn_left(20)
                                while(is_far(multiranger.front)):
                                    do = 0
                                print("found left\n\n\n\n")
                                mc.start_turn_right(20)
                                time.sleep(2)
                                xD = world.drone.x
                                yD = world.drone.y
                                yawD = world.drone.yaw_deg
                                dist = multiranger.front
                                print("Drone X: {}, Drone Y: {}, Angle:{}, Distance: {}".format(xD,yD,yawD,dist))
                                
                                leftNode = detect_to_Node(xD,yD,yawD,dist)

                                while(is_far(multiranger.front)):
                                    do = 0

                                print("found right\n\n\n\n")
                                mc.start_turn_left(20)
                                time.sleep(1)
                                xD = world.drone.x
                                yD = world.drone.y
                                yawD = world.drone.yaw_deg
                                dist = multiranger.front
                                rightNode = detect_to_Node(xD,yD,yawD,dist)
                                print("Drone X: {}, Drone Y: {}, Angle:{}, Distance: {}".format(xD,yD,yawD,dist))
                                mc.stop
                                world.createWallBlock(leftNode,rightNode)
                            while(world.checkPaths()):
                                world.updateGraph()
                            world.graph.updateWeights(world.currentGoal)


            #We land when the MotionCommander goes out of scope

    # done = False
    # while(not done):
    #     world.combine()
    #     done = True
    #     for i in world.blocks:
    #         if(((i.width*i.height)/(world.width*world.height))<10/(world.width*world.height) and i.flyable):
    #             done = False
    while(world.checkPaths()):
        world.updateGraph()

    world.graph.updateWeights(world.blocks[0].node)
    #world.adjustForSize()
    # print("testing for bad paths\n")
    # while(world.checkPaths()):
    #     world.updateGraph()
    # print("final results\n")


    #world.print()

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