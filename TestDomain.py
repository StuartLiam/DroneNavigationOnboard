import Virtual as vr
import GraphDomain as gd
import LineHelp as lh
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import matplotlib.patches as patches
import numpy as np
HORIZONTAL = 1
VERITCAL = 2

print("starting tests\n")



print ("Creating World\n")
world = vr.World(0,0,300,300,[gd.Node(200,200)])
world.drone.x = 1
world.drone.y = 1
world.print()

print("adding in a line at 5,5  10,5")
world.createWallBlock(gd.Node(50,150),gd.Node(150,150))

# print("updating the graph for the split\n")
# world.updateGraph()

# print("testing for bad paths\n")
# while(world.checkPaths()):
#     world.updateGraph()

# print("adding in a line at 20,20  20,10")
# world.createWallBlock(gd.Node(150,50),gd.Node(150,200))

# world.createWallBlock(gd.Node(50,200),gd.Node(150,200))

world.updateGraph()

print("testing for bad paths\n")
while(world.checkPaths()):
    world.updateGraph()



print("printing results\n")
world.print()


done = False
while(not done):
    world.combine()
    done = True
    for i in world.blocks:
        if(((i.width*i.height)/(world.width*world.height))<0.1 and i.flyable):
            done = False
while(world.checkPaths()):
    world.updateGraph()

world.graph.updateWeights(world.blocks[0].node)
world.adjustForSize()
# print("testing for bad paths\n")
# while(world.checkPaths()):
#     world.updateGraph()
# print("final results\n")

world.print()




#lh.intersect(gd.Node(1,1),gd.Node(5,1),gd.Node(3,1),gd.Node(5,5))


fig2 = plt.figure()
ax2 = fig2.add_subplot(111, aspect='equal')
ax2.margins(x=300,y=300)
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


