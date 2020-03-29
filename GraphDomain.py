

class Node:
    def __init__(self,x,y):
        print("node created")
        self.x = x
        self.y = y

    def print(self):
        print("Node at {},{}".format(self.x,self.y))



class Edge:
    def __init__(self,nodeOne,nodeTwo,weight = 0):
        self.nodeOne = nodeOne
        self.nodeTwo = nodeTwo
        self.weight = weight

    def print(self):
        print("edge between 2 following nodes: ")
        self.nodeOne.print()
        self.nodeTwo.print()
        print("\n")

        
class Graph:
    def __init__(self,nodes = None, edges = None):
        self.nodes = nodes if (nodes != None) else []
        self.edges = edges if (edges != None) else []

    def print(self):
        print("Has nodes: \n")
        for i in self.nodes:
            i.print()
        print("Has edges: \n")
        for i in self.edges:
            i.print()

    