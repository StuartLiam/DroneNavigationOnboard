import Virtual as vr


HORIZONTAL = 1
VERITCAL = 2

print("starting tests\n")

print ("Creating World\n")
world = vr.World(0,0,30,30)
world.print()

print ("splitting the block\n")
world.split(10,10,VERITCAL,True)
world.split(5,5,HORIZONTAL,True)
world.updateGraph()

print("new blocks are")
world.print()

print("testing for bad paths")
world.checkPaths()
