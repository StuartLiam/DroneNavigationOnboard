import matplotlib.pyplot as plt
import numpy as np
import random

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
    def __init__(self, x, y, width, height, color = None, flyable = False):
        super().__init__( x, y, width, height)
        self.color = color
        # if ( color is not None ):
        #     self.patch = plt.Rectangle((self.x, self.y), self.width, self.height, facecolor=color, edgecolor='#202020')
        self.flyable = flyable
    def inside(self,x,y):
        return True if (self.x < x and self.x + self.width > x and self.y < y and self.y + self.height > y ) else False

    def print(self):
        super().print()
        print("Is flyable {}\n".format(self.flyable))



class World(Rectangle):
    def __init__(self, x, y, width, height, color = None ):
        super().__init__( x, y, width, height)
        # if ( color is not None ):
        #     self.patch = plt.Rectangle((self.x, self.y), self.width, self.height, facecolor=color, edgecolor='#202020')
        self.blocks = [Block(self.x,self.y,self.width,self.height,True)]
        self.drone = VirDrone(x,y)
    def split(self, x, y, dir, type):
        
        for i in self.blocks:
            if (i.inside(x,y)):
                if dir == VERITCAL:
                    self.blocks.append(Block(x,i.y,((i.x + i.width)-x),i.height))
                    i.width = x - i.x
                if dir == HORIZONTAL:
                    self.blocks.append(Block(i.x,y,i.width,((i.y + i.height)-y)))
                    i.height = y - i.y

    def updateGraph(self):
        print("updating the graph")



    def print(self):
        super().print()
        print("World contains the following blocks:\n")
        for i in self.blocks:
            i.print()
            plt.Rectangle((i.x, i.y), i.width, i.height, edgecolor='#202020')
        plt.show()

