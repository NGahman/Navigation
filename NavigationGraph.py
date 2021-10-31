import NavigationPath
import math
import copy
'''graph = {'A': ['B','D'],
         'B': ['C','D'],
         'C': ['A','E'],
         'D':['C','E'],
         'E':['A','B']
         }'''
class Node:
    def __init__(self,coords):
        self.coords = coords
        self.f = 0
        self.g = 0
        self.h = 0
        self.parent = None
        self.origin = None
    def UpdateStats(self,g,h):
        self.g = g
        self.h = h
        self.f = g + h
    def GetStats(self):
        return self.f,self.g,self.h
    def EqualCoords(self,other):
        if self.coords == other.coords:
            return True
        return False
    def EqualCoordsEqualParent(self,other):
        if self.coords == other.coords and self.parent == other.parent:
            return True
        return False
    
    def UpdateParent(self,parent):
        self.parent = parent
    def GetParent(self):
        return self.parent
    def UpdateOrigin(self,origin):
        self.origin = origin
    def GetOrigin(self):
        return self.origin
    
    def __lt__(self,other):
        if self.f < other.f:
            return True
        return False
    def __gt__(self,other):
        if self.f > other.f:
            return True
        return False
    
class Edge:
    def __init__(self,EndNode,weight):
        self.EndNode = EndNode
        self.weight = weight
    def GetEdge(self):
        return self.EndNode, self.weight

class Graph:
    def __init__(self):
        self.graph = {}

    def AddNode(self,Node):
        for i in self.graph:
            if Node.EqualCoords(i):
                return False
        self.graph[Node] = []
        return True
    def AddEdge(self,Node,Edge):
        for i in self.graph[Node]:
            if i.EndNode == Edge.EndNode:
                return False
        NodeValue = self.graph.get(Node)
        NodeValue.append(Edge)
        self.graph[Node] = NodeValue
        return True
    def RemoveEdge(self,Node,Edge):
        self.graph[Node].remove(Edge)
    def RemoveNode(self,Node):
        del self.graph[Node]
        for i in self.graph:
            for j in self.graph[i]:
                if j.EndNode = Node:
                    self.graph[i].remove(j)
    def getNode(self,location):
        for i in self.graph:
            if i.coords == location:
                return i
        return None
    def ResetGraph(self):
        self.graph = {}
        

class NavigationGraph:

    def __init__(self,CurrentLocation,Destination):
        self.currentlocation = CurrentLocation
        self.destination = Destination
        self.paths = []
        self.graph = Graph()

    def BuildGraph(self):
        #Needs Google Maps API to work
        NodeA = Node([0.0,2.0])
        NodeB = Node([1.0,1.0])
        NodeC = Node([0.0,1.0])
        NodeD = Node([0.0,0.0])
        NodeE = Node([0.0,-1.0])
        
        EdgeAB = Edge(NodeB,1)
        EdgeAD = Edge(NodeD,3)
        EdgeBC = Edge(NodeC,1)
        EdgeBD = Edge(NodeD,1)
        EdgeCA = Edge(NodeA,1)
        EdgeCE = Edge(NodeE,2)
        EdgeDC = Edge(NodeC,1)
        EdgeDE = Edge(NodeE,1)
        EdgeEA = Edge(NodeA,1)
        EdgeEB = Edge(NodeB,1)
        
        self.graph.AddNode(NodeA)
        self.graph.AddNode(NodeB)
        self.graph.AddNode(NodeC)
        self.graph.AddNode(NodeD)
        self.graph.AddNode(NodeE)
        
        self.graph.AddEdge(NodeA,EdgeAB)
        self.graph.AddEdge(NodeA,EdgeAD)
        self.graph.AddEdge(NodeB,EdgeBC)
        self.graph.AddEdge(NodeB,EdgeBD)
        self.graph.AddEdge(NodeC,EdgeCA)
        self.graph.AddEdge(NodeC,EdgeCE)
        self.graph.AddEdge(NodeD,EdgeDC)
        self.graph.AddEdge(NodeD,EdgeDE)
        self.graph.AddEdge(NodeE,EdgeEA)
        self.graph.AddEdge(NodeE,EdgeEB)
        
    def Heuristic(self,Node):
        return math.sqrt((Node.location[0]-self.Destination[0])**2 + (Node.location[1]-self.Destination[1])**2)
    
    def Navigate(self,paths):
        #TBA: A* should work here with edits for multiple paths?
        #TBA: Find for copy.deepcopy nodes find equivalent node in graph via EqualCoords
        Goal = copy.deepcopy(self.graph.getNode(self.destination))
        Start = copy.deepcopy(self.graph.getNode(self.currentlocation))
        OpenList = []
        ClosedList = []
        ParentLists = []
        counter = 0
        OpenList.append(Start)
        while counter < paths:
            
            if len(OpenList) == 0:
                break
            OpenList.sort()
            while OpenList[0] == None:
                del OpenList[0]
            CurrentNode = OpenList[0]
            del OpenList[0]
            ClosedList.append(CurrentNode)
            CurrentF,CurrentG,CurrentH = CurrentNode.GetStats()
            
            if CurrentNode.EqualCoords(Goal):
                ParentList = []
                if counter < paths:
                    while CurrentNode.GetParent() is not None:
                        ParentList.append(CurrentNode)
                        CurrentNode = CurrentMove.GetParent()
                    ParentList.append(CurrentNode)
                    ParentLists.append((CurrentG,ParentList[::-1]))
                    counter += 1
            print(OpenList)
            print(CurrentNode)
            for i in self.graph.graph[CurrentNode]:
                ListCheck = True
                E = copy.deepcopy(i.EndNode)
                E.UpdateParent(CurrentNode)
                E.UpdateStats(CurrentG+i.weight,Heuristic(i.EndNode))
                for j in ClosedList:
                    if E.EqualCoordsEqualParents(j):
                        ListCheck = False
                if j in OpenList:
                    if E.EqualCoordsEqualParents(j):
                        ListCheck = False
                        i.UpdateStats(min(j.g,E.g),j.h)
                if ListCheck == True:
                    OpenList.append(E)
        self.paths = []
        for i in ParentLists:
            Path = NavigationPath(i[1],i[0],i[0],i[0])
            self.paths.append(Path)

            
            

    def ChangeDestination(self,Destination):
        self.destination = Destination
        self.paths = []

    def GetPaths(self):
        return self.paths

    def FollowPath(Path,NumInstructions):
        Instructions = Path.GetInstructions()
        for i in range(NumInstructions):
            instruction = Instructions[0]
            #TBA: Actually work with instructions once API is completed
            
NG = NavigationGraph([0.0,0.0],[0.0,-1.0])
NG.BuildGraph()
NG.Navigate(1)
for i in NG.GetPaths():
    print(i.GetInstructions())

