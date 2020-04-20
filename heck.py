import math
import VirtualFix as vr
import GraphDomain as gd

world = vr.World(0,0,3,3,[gd.Node(1.5,2)])
world.drone.x = 1
world.drone.y = 1

def atNode(node):
    return abs(world.drone.x - node.x) < 0.2 and abs(world.drone.y - node.y) < 0.2


print(atNode(world.getDroneBlock().node))