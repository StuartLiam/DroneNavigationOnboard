import Virtual as vr


HORIZONTAL = 1
VERITCAL = 2

print("starting tests\n")

print ("Creating World\n")
world = vr.World(0,0,30,30)
world.print()

print ("splitting the block\n")
world.split(10,10,VERITCAL,True)
world.split(5,5,HORIZONTAL,False)
world.updateGraph()

print("new blocks are\n")
world.print()

print("testing for bad paths\n")
world.checkPaths()

print("updating the graph for the split\n")
world.updateGraph()

print("printing results\n")
world.print()

