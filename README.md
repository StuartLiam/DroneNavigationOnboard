
- Create virtual environment
- Create virtual box
- Take off
- If goal in same virtual box path to goal

- Else Path to next box in graph

- If path found new obsticle
    split virtual box
    new box obsticle

if path found new open space
    split virtual box
    new box flyable



path(drone, goal)

while drone not at goal
    if drone same block as goal
        go_to goal
        if(block)
            scan
            estimate position
    if drone not same block
        if drone at node
            go to best node with edge
        else go to node
            if(block)
                scan
                estimate position

while not drone.atNode(goal)
    if droneBlock is goalBlock
        goto(drone, currentGoal)
        if(block)
            world.createBlock(scan(drone))
             drone.x = data['kalman.varPX']
             drone.y = data['kalman.varPX']
    else
        if(drone.atNode(droneBlock.node))
            bestNext = world.bestNext()
            goto(drone, bestNext)
            if(block)
                world.createBlock(scan(drone))
                drone.x = data['kalman.varPX']
                drone.y = data['kalman.varPX']
        else
            goto(drone, droneBlock.node)
            if(block)
                world.createBlock(scan(drone))
                drone.x = data['kalman.varPX']
                drone.y = data['kalman.varPX']