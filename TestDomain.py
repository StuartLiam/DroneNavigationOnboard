import VirtualFix as vr
import GraphDomain as gd
import LineHelp as lh
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import matplotlib.patches as patches
import numpy as np
import pylab as pl
from matplotlib import collections  as mc
import numpy as np
import math
HORIZONTAL = 1
VERITCAL = 2

print("starting tests\n")


# def path(motioncommander, drone,node):
#     d = math.sqrt(((drone.x-node.x)**2)+((drone.y-node.y)**2))

#     angle = math.atan(node.y - drone.y) if (node.x-drone.x == 0) else math.atan((node.y-drone.y)/(node.x-drone.x))
#     angle = math.degrees(angle)
#     angle = angle + 90 if (node.x < drone.x and node.y > drone.y) else angle
#     angle = angle + 180 if (node.x < drone.x and node.y < drone.y) else angle
#     angle = angle + 270 if (node.x >drone.x and node.y < drone.y) else angle
#     return [d,angle]

# path = path(0,gd.Node(0,0),gd.Node(10,10))

# print("length {} at angle {}".format(path[0],path[1]))
print ("Creating World\n")
world = vr.World(0,0,6,6,[gd.Node(5,5)])
world.drone.x = 0.5
world.drone.y = 0.5
world.print()

#world.split(2.5,2.5,VERITCAL,True,True)

print("adding in a line at 5,5  10,5")
world.createWallBlock(gd.Node(4,2),gd.Node(4,4))
world.createWallBlock(gd.Node(1,1),gd.Node(1,2))
#world.createWallBlock(gd.Node(2,3),gd.Node(3,3))
#world.createWallBlock(gd.Node(0,5),gd.Node(3,5))

# print("updating the graph for the split\n")
# world.updateGraph()

# print("testing for bad paths\n")
# while(world.checkPaths()):
#     world.updateGraph()

# print("adding in a line at 20,20  20,10")
#world.createWallBlock(gd.Node(3,3),gd.Node(3,4))

#

# print("testing for bad paths\n")
#while(world.checkPaths()):

print("\n\nupdating graph\n\n")
world.updateGraph()


# world.updateGraph()
# print("printing results\n")
# world.print()


# done = False
# while(not done):
#     world.combine()
#     done = True
#     for i in world.blocks:
#         if(((i.width*i.height)/(world.width*world.height))<10/(world.width*world.height) and i.flyable):
#             done = False
while(world.checkPaths()):
    world.updateGraph()
# for i in range(4):
#     world.checkPaths()
#     world.updateGraph()

# world.updateGraph()
#world.graph.updateWeights(world.blocks[0].node)
# #world.adjustForSize()
# print("testing for bad paths\n")
# while(world.checkPaths()):
#     world.updateGraph()
# print("final results\n")
#world.checkPaths()
world.updateGraph()
while(world.checkPaths()):
    world.updateGraph()
world.graph.updateWeights(world.getGoalBlock().node)
world.adjustForSize()
world.print()

path = world.graph.bestPath(world.getDroneBlock().node, world.getGoalBlock().node)
path.append(gd.Edge(world.getGoalBlock().node,world.currentGoal))


#lh.intersect(gd.Node(1,1),gd.Node(5,1),gd.Node(3,1),gd.Node(5,5))


fig, ax = pl.subplots()

ax = fig.add_subplot(111, aspect='equal')
#ax.margins(x=6,y=6)
for i in world.blocks:
    ax.add_patch(
        patches.Rectangle(
            (i.x, i.y),
            i.width,
            i.height,
            fill=False if (i.flyable) else True       # remove background
        ) )
ax.autoscale()
ax.margins(0.1)
# for i in world.graph.edges:
#     lc = mc.LineCollection([[(i.nodeOne.x,i.nodeOne.y),(i.nodeTwo.x,i.nodeTwo.y)]], colors=(1, 0, 0, 1), linewidths=2)
#     ax.add_collection(lc)
for i in path:
    lc = mc.LineCollection([[(i.nodeOne.x,i.nodeOne.y),(i.nodeTwo.x,i.nodeTwo.y)]], colors=(1, 0, 0, 1), linewidths=2)
    ax.add_collection(lc)
plt.scatter(world.currentGoal.x,world.currentGoal.y)
#plt.scatter(world.graph.nodes[0].x,world.graph.nodes[0].y)
plt.show()


