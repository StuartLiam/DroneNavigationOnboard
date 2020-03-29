import matplotlib.pyplot as plt
import numpy as np
import random
import GraphDomain as gd
import LineHelp as lh
HORIZONTAL = 1
VERITCAL = 2

class VirDrone:
    def __init__(self,x,y):
        self.x = x
        self.y = y

        

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
        self.node = gd.Node(x+(width//2),y+(height//2))
    def inside(self,x,y):
        return True if (self.x < x and self.x + self.width > x and self.y < y and self.y + self.height > y ) else False

    def print(self):
        super().print()
        print("Is flyable {}\n".format(self.flyable))

    def getLines(self):
        return [
                [gd.Node(self.x,self.y),gd.Node(self.x,self.y+self.height)],
                [gd.Node(self.x,self.y),gd.Node(self.x+self.width,self.y)],
                [gd.Node(self.x+self.width,self.y+self.height),gd.Node(self.x,self.y+self.height)],
                [gd.Node(self.x+self.width,self.y+self.height),gd.Node(self.x+self.width,self.y)]
               ]


class World(Rectangle):
    def __init__(self, x, y, width, height, color = None ):
        super().__init__( x, y, width, height)
        # if ( color is not None ):
        #     self.patch = plt.Rectangle((self.x, self.y), self.width, self.height, facecolor=color, edgecolor='#202020')
        self.blocks = [Block(self.x,self.y,self.width,self.height,True)]
        self.drone = VirDrone(x,y)
        self.graph = gd.Graph()
    def split(self, x, y, dir, flyable):
        
        for i in self.blocks:
            if (i.inside(x,y)):
                if dir == VERITCAL:
                    self.blocks.append(Block(x , i.y , ((i.x + i.width)-x) , i.height, flyable) )
                    i.width = x - i.x
                    i.node.x = i.x + (i.width//2)
                if dir == HORIZONTAL:
                    self.blocks.append(Block( i.x , y , i.width , ((i.y + i.height)-y), flyable) )
                    i.height = y - i.y
                    i.node.y = i.y + (i.height//2)
            

    def updateGraph(self):
        print("updating the graph")
        self.graph.edges = []
        self.graph.nodes = []
        for i in self.blocks:
            for j in self.blocks:
                if(i.x+i.width == j.x   #shares same x value
                and ((j.y >= i.y and j.y <= i.y + i.height) or (i.y >= j.y and i.y <= j.y + j.height)) # 1st y is inbetween 2nd or 2nd is between 1st
                and(j.flyable==True and i.flyable==True)): #both are flyable
                    self.graph.edges.append(gd.Edge(i.node,j.node))

                if(i.y + i.height == j.y 
                and ((j.x >= i.x and j.x <= i.x + i.width) or (i.x >= j.x and i.x <= j.x + j.width))
                and(j.flyable==True and i.flyable==True)):
                    self.graph.edges.append(gd.Edge(i.node,j.node))
            
            self.graph.nodes.append(i)

    def checkPaths(self):

        for i in self.graph.edges:
            for j in self.blocks:
                lines = j.getLines()
                for k in lines:
                    if(lh.intersect(i.nodeOne,i.nodeTwo,k[0],k[1])):
                        print("Crosses a bad block")



    def print(self):
        super().print()
        print("World contains the following blocks:\n")
        for i in self.blocks:
            i.print()
            plt.Rectangle((i.x, i.y), i.width, i.height, edgecolor='#202020')

        print("World has the following graph : \n")
        self.graph.print()
        

