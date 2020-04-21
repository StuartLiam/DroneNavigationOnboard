import matplotlib.pyplot as plt
import numpy as np
import random
import GraphDomain as gd
import LineHelp as lh
from shapely.geometry import Polygon
from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.geometry import box
HORIZONTAL = 1
VERITCAL = 2

class VirDrone:
    def __init__(self,x,y,world):
        self.x = x
        self.y = y
        self.world = world
        self.yaw_deg = 90

# The base class for pokygons
class Rectangle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    def print(self):
        print("Starts at {} , {} and is {} long by {} tall".format(self.x,self.y,self.width,self.height))


class Block(Rectangle):
    def __init__(self, x, y, width, height, flyable):
        super().__init__( x, y, width, height)

        self.flyable = flyable
        self.node = gd.Node(x+(width/2),y+(height/2),self)

    def inside(self,x,y):
        return True if (self.x < x and self.x + self.width > x and self.y < y and self.y + self.height > y ) else False

    def print(self):
        super().print()
        print("Is flyable {}\n".format(self.flyable))

    def getLines(self):
        return [
                [gd.Node(self.x,self.y),gd.Node(self.x,self.y+self.height)],
                [gd.Node(self.x,self.y),gd.Node(self.x+self.width,self.y)],
                [gd.Node(self.x,self.y+self.height),gd.Node(self.x+self.width,self.y+self.height)],
                [gd.Node(self.x+self.width,self.y),gd.Node(self.x+self.width,self.y+self.height)]
               ]


class World(Rectangle):
    def __init__(self, x, y, width, height, goals = None):
        super().__init__( x, y, width, height)
        # if ( color is not None ):
        #     self.patch = plt.Rectangle((self.x, self.y), self.width, self.height, facecolor=color, edgecolor='#202020')
        self.blocks = [Block(self.x,self.y,self.width,self.height,True)]
        self.drone = VirDrone(x,y,self)
        self.graph = gd.Graph()
        self.goals = goals
        self.currentGoal = None if (goals is None) else goals[0]


    def split(self, x, y, dir, flyOne, flyTwo):
        
        for i in self.blocks:
            if (i.inside(x,y) and i.flyable):
                if dir == VERITCAL:
                    self.blocks.append(Block(x , i.y , ((i.x + i.width)-x) , i.height, flyTwo) )
                    i.width = x - i.x
                    i.node.x = i.x + (i.width/2)
                    i.flyable = flyOne
                if dir == HORIZONTAL:
                    self.blocks.append(Block( i.x , y , i.width , ((i.y + i.height)-y), flyTwo) )
                    i.height = y - i.y
                    i.node.y = i.y + (i.height/2)
                    i.flyable = flyOne
            

    def updateGraph(self):
        #print("updating the graph")
        self.graph.edges = []
        self.graph.nodes = []
        for i in self.blocks:
            for j in self.blocks:
                if(i.inside(j.node.x,j.node.y) and j.flyable==False):
                    i.flyable = False

        for i in self.blocks:
            for j in self.blocks:
                if(i.x == j.x + j.width):
                    if((i.y >= j.y and i.y <= j.y + j.height) or (j.y >= i.y and j.y <= i.y + i.height)):
                        if(i.flyable and j.flyable):
                            self.graph.edges.append(gd.Edge(i.node,j.node))

                if(i.y == j.y + j.height):
                    if((i.x >= j.x and i.x <= j.x + j.width) or (j.x >= i.x and j.x <= i.x + i.width)):
                        if(i.flyable and j.flyable):
                            self.graph.edges.append(gd.Edge(i.node,j.node))

        

            self.graph.nodes.append(i.node)#TODO check valid change from i

        # for i in self.blocks:
        #     for j in self.blocks:
        #         if(i.x+i.width == j.x   #shares same x value
        #         and ((j.y >= i.y and i.y + i.height - j.y > 0.1) or (i.y >= j.y and j.y + j.height - i.y > 0.1)) # 1st y is inbetween 2nd or 2nd is between 1st
        #         and(j.flyable==True and i.flyable==True)): #both are flyable
        #             self.graph.edges.append(gd.Edge(i.node,j.node))

        #         if(i.y + i.height == j.y 
        #         and ((j.x >= i.x and i.x + i.width - j.x > 0.1) or (i.x >= j.x and j.x + j.width - i.x < 0.1))
        #         and(j.flyable==True and i.flyable==True)):
        #             self.graph.edges.append(gd.Edge(i.node,j.node))
            
        #     self.graph.nodes.append(i.node)#TODO check valid change from i

    def checkPaths(self):
        FoundOne = False
        for i in self.graph.edges:
            line_i = LineString([(i.nodeOne.x,i.nodeOne.y),(i.nodeTwo.x,i.nodeTwo.y)])
            for j in self.blocks:
                xy = [[j.x, j.y], [j.x, j.y + j.height], 
                    [j.x + j.width, j.y + j.height], [j.x + j.width, j.y]]
                polygon_j = Polygon(xy)
                #cross = polygon_j.intersects
                if( polygon_j.intersects(line_i) and j.flyable == False):

                    print("Crosses a bad block")
                    print("edge: ")
                    i.print()
                    print("crosses")
                    j.print()
                    FoundOne = True
                    areaOne = i.nodeOne.parent.width*i.nodeOne.parent.height
                    areaTwo = i.nodeTwo.parent.width*i.nodeTwo.parent.height
                    
                    #s = 0 if (i.nodeTwo.x==i.nodeOne.x) else (i.nodeTwo.y-i.nodeOne.y)/(i.nodeTwo.x-i.nodeOne.x)

                    if (areaOne >= areaTwo):
                        if(i.nodeOne.parent.width >= i.nodeOne.parent.height):
                            self.split(i.nodeOne.x, i.nodeOne.y,VERITCAL,True,True)
                        if(i.nodeOne.parent.width < i.nodeOne.parent.height):
                            self.split(i.nodeOne.x, i.nodeOne.y,HORIZONTAL,True,True)

                    if (areaTwo >= areaOne):
                        if(i.nodeTwo.parent.width >= i.nodeTwo.parent.height):
                            self.split(i.nodeTwo.x, i.nodeTwo.y,VERITCAL,True,True)
                        if(i.nodeTwo.parent.width < i.nodeTwo.parent.height):
                            self.split(i.nodeTwo.x, i.nodeTwo.y,HORIZONTAL,True,True)
                    return True
        return FoundOne
    
    def createWallBlock(self,nodeOne, nodeTwo):
        #print("correctly called \n\n\n\n")
        minX = min(nodeOne.x,nodeTwo.x)
        maxX = max(nodeOne.x,nodeTwo.x)
        minY = min(nodeOne.y,nodeTwo.y)
        maxY = max(nodeOne.y,nodeTwo.y)
        midX = minX + ((maxX-minX)/2)
        midY = minY + ((maxY-minY)/2)
        safety = 0.2

        block = Block(0,0,0,0,False)

        if(abs(nodeOne.x-nodeTwo.x)<1):
            #print("vertical")
            block = Block(midX - safety, minY - safety, 2*safety, maxY-minY + 2*safety, False)
            self.blocks.append(block)            
            xy = [[block.x, block.y], [block.x, block.y + block.height], 
                [block.x + block.width, block.y + block.height], [block.x + block.width, block.y]]
            polygon_shape = Polygon(xy)

            for i in self.blocks:
                #print("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]

                block_shape = Polygon(xy2)
            # The intersection
            
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    try:
                        x,y = j.exterior.coords.xy
                    #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                    
                        self.split(min(x),i.node.y,VERITCAL,True,True)
                    except:
                        #print("k")
                        do = 10


            for i in self.blocks:
                #rint("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]
                block_shape = Polygon(xy2)
            # The intersection
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    
                    try:
                        x,y = j.exterior.coords.xy
                        #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                        self.split(i.node.x,min(y),HORIZONTAL,True,True)
                    except:
                        #print("k")
                        do = 10

            
            for i in self.blocks:
                #print("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]
                block_shape = Polygon(xy2)
            # The intersection
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    try:
                        x,y = j.exterior.coords.xy
                    #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                    
                        self.split(i.node.x,max(y),HORIZONTAL,True,True)
                    except:
                        #print("k")
                        do = 10
            
            for i in self.blocks:
                #print("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]
                block_shape = Polygon(xy2)
            # The intersection
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    try:
                        x,y = j.exterior.coords.xy
                    #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                    
                        self.split(max(x),i.node.y,VERITCAL,True,True)
                    except:
                        #print("k")
                        do = 10

          

        else:
            #print("horizontal")

            block = Block(minX-safety,midY-safety,maxX-minX+(2*safety),2*safety,False)
            self.blocks.append(block)            
            xy = [[block.x, block.y], [block.x, block.y + block.height], 
                 [block.x + block.width, block.y + block.height], [block.x + block.width, block.y]]
            polygon_shape = Polygon(xy)
            # Example grid cell

            for i in self.blocks:
               # print("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]
                block_shape = Polygon(xy2)
            # The intersection
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    
                    try:
                        x,y = j.exterior.coords.xy
                        #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                        self.split(i.node.x,min(y),HORIZONTAL,True,True)
                    except:
                        #print("k")
                        do = 10


            
            for i in self.blocks:
                #print("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]
                block_shape = Polygon(xy2)
            # The intersection
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    try:
                        x,y = j.exterior.coords.xy
                    #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                    
                        self.split(min(x),i.node.y,VERITCAL,True,True)
                    except:
                        #print("k")
                        do = 10

            for i in self.blocks:
                #print("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]
                block_shape = Polygon(xy2)
            # The intersection
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    try:
                        x,y = j.exterior.coords.xy
                    #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                    
                        self.split(max(x),i.node.y,VERITCAL,True,True)
                    except:
                        #print("k")
                        do = 10

            
            for i in self.blocks:
                #print("new block \n\n\n")
                xy2 = [[i.x, i.y], [i.x, i.y + i.height], 
                 [i.x + i.width, i.y + i.height], [i.x + i.width, i.y]]
                block_shape = Polygon(xy2)
            # The intersection
                j = polygon_shape.intersection(block_shape)
                if (type(j) is Polygon):
                    try:
                        x,y = j.exterior.coords.xy
                    #print("splitting at {},{}\n\n".format(i.node.x,min(y)))
                    
                        self.split(i.node.x,max(y),HORIZONTAL,True,True)
                    except:
                        #print("k")
                        do = 10


                # for k,l in zip(x,y):
                #     print("{},{}\n".format(k,l))
                    
                
            


        #horizontal

            

    def combine(self):
        # found = True
        # while(found):
        found = False
        for i in self.blocks:
            for j in self.blocks:
                if(i.y == j.y and i.height == j.height and i.x+i.width == j.x and j.flyable and i.flyable):
                    i.width = i.width + j.width
                    self.blocks.remove(j)
                    found = True
                elif(i.x == j.x and i.width == j.width and i.y+i.height == j.y and j.flyable and i.flyable):
                    i.height = i.height + j.height
                    self.blocks.remove(j)
                    found = True

        #self.updateGraph()

    def adjustForSize(self):
        for i in self.blocks:
            if(i.flyable):
                i.node.weight = i.node.weight * ((i.width*i.height))

    def getDroneBlock(self):
        for i in self.blocks:
            if(i.inside(self.drone.x,self.drone.y)):
                return i
        return None

    def getGoalBlock(self):
        for i in self.blocks:
            if(i.inside(self.currentGoal.x,self.currentGoal.y)):
                return i
        return None

    def bestNode(self,node):
        bestNode = None
        for i in self.graph.edges:
            if(i.nodeOne is node):
                if((bestNode is None) or i.nodeTwo.weight < bestNode.weight):
                    bestNode = i.nodeTwo    
        return bestNode

    def print(self):
        super().print()
        print("World contains the following blocks:\n")
        for i in self.blocks:
            i.print()
            plt.Rectangle((i.x, i.y), i.width, i.height, edgecolor='#202020')

        print("World has the following graph : \n")
        self.graph.print()
        

