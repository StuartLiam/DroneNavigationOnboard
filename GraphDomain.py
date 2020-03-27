from Virtual import World



class Node:
    def __init__(self):
        print("node created")

class Edge:
    def __init__(self,nodeOne,nodeTwo,weight = 0):
        self.nodeOne = nodeOne
        self.nodeTwo = nodeTwo
        self.weight = weight

class Graph:
    def __init__(self,nodes = None, edges = None):
        self.nodes = nodes if (nodes != None) else []
        self.edges = edges if (edges != None) else []
    