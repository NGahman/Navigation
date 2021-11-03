from NavigationPath import NavigationPath
import math
import copy
class Node
    def __init__(self,coords):
        #Coords is the primary distinguishing characteristic of a node, no two nodes should have the same lat/long coords.
        #F, G and H are used for the A* search algorithm used in Navigate(n).  G is the weight of all edges taken from the start to the current node, and H is the heuristic (distance between node's coords and destination's coords).
        #Parent is used to trace back the exact path taken from start to goal.
        #Origin is used to find the node in the given graph, as all Nodes created in Navigate(n) are copies of nodes from the graph to ensure the graph is not effected.  However, these copied nodes are obviously not in the graph, so Origin is needed to find the edges of a given node.
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

    #EqualCoords checks if the two nodes have identical coords and if so returns true.
    def EqualCoords(self,other):
        if self.coords == other.coords:
            return True
        return False

    #EqualCoordsEqualParent checks if the two nodes have identical coords and the same parent node and if so returns true.
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

    #All nodes in a list are sorted using their F value. (there is not a __eq__ because the dictionary requires that to be left blank for a given node to be a viable key)
    def __lt__(self,other):
        if self.f < other.f:
            return True
        return False
    def __gt__(self,other):
        if self.f > other.f:
            return True
        return False
    
class Edge:
    #Note: weight might have to be changed to miles and mph down the line once BuildGraph is completed, so that NavigationPath's Miles, Kilometers, and Time variables actually mean something
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
    #Removes the node from the graph and all edges that point to said node.
    def RemoveNode(self,Node):
        del self.graph[Node]
        for i in self.graph:
            for j in self.graph[i]:
                if j.EndNode == Node:
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

    #Returns the Elucidian Distance between the Node and NavigationGraph's destination variable.
    def Heuristic(self,Node):
        return math.sqrt((Node.coords[0]-self.destination[0])**2 + (Node.coords[1]-self.destination[1])**2)
    
    def Navigate(self,paths):
        #Instantiates goal and start nodes based on the destination and current location, respectively
        Goal = copy.deepcopy(self.graph.getNode(self.destination))
        Goal.UpdateOrigin(self.graph.getNode(self.destination))
        Start = copy.deepcopy(self.graph.getNode(self.currentlocation))
        Start.UpdateOrigin(self.graph.getNode(self.currentlocation))
        #Sets up the loop
        OpenList = []
        ClosedList = []
        ParentLists = []
        counter = 0
        OpenList.append(Start)
        while counter < paths:
            #If at any point the OpenList has no Nodes left, the while loop breaks.  This is to make sure if the # of unique paths to the goal are less than the variable paths, Navigate doesn't break
            if len(OpenList) == 0:
                break
            #Sorts the list, takes the first Node from the list and deletes said node from OpenList
            OpenList.sort()
            while OpenList[0] == None:
                del OpenList[0]
            CurrentNode = OpenList[0]
            del OpenList[0]
            #Adds deleted node to the closed list, and gets the F, G and H values from the current Node
            ClosedList.append(CurrentNode)
            CurrentF,CurrentG,CurrentH = CurrentNode.GetStats()

            #If the given node is Goal and the number of paths already found are less than the given variable paths, the entire path of nodes is added to ParentList, which in turn is added to ParentLists.
            if CurrentNode.EqualCoords(Goal):
                ParentList = []
                if counter < paths:
                    while CurrentNode.GetParent() is not None:
                        ParentList.append(CurrentNode)
                        CurrentNode = CurrentNode.GetParent()
                    ParentList.append(CurrentNode)
                    ParentLists.append((CurrentG,ParentList[::-1]))
                    counter += 1

            #For every unique edge CurrentNode has, it checks to see if the Node said edge has is already in OpenList or ClosedList and comes from the same parent/edge.        
            for i in self.graph.graph[CurrentNode.GetOrigin()]:
                ListCheck = True
                E = copy.deepcopy(i.EndNode)
                E.UpdateParent(CurrentNode)
                E.UpdateOrigin(i.EndNode)
                E.UpdateStats(CurrentG+i.weight,self.Heuristic(i.EndNode))
                #If there is already a child node from that edge in ClosedList, nothing further is done.
                for j in ClosedList:
                    if E.EqualCoordsEqualParent(j):
                        ListCheck = False
                #If there is already a child node from that edge in OpenList, the node in OpenList is updated to be either its old g value, or the g value obtained currently, whichever is less.
                #This ensures the node in question will always exemplify the fastest path to a given edge.
                if j in OpenList:
                    if E.EqualCoordsEqualParent(j):
                        ListCheck = False
                        i.UpdateStats(min(j.g,E.g),j.h)
                #If the given node from the edge is in neither OpenList or ClosedList, the node is appended to OpenList.
                if ListCheck == True:
                    OpenList.append(E)
                    
                    #print(CurrentG)
                    #print(CurrentNode.coords,E.coords,E.g)
        #Once the number of unique paths to the destination or the fastest paths number of unique paths is found, whichever is less, these paths stored in ParentLists are converted to NavigationPaths and appended to self.paths.
        #Before this, self.paths is set to no paths, so that the unique paths in Navigate() do not stack.
        self.paths = []
        for i in ParentLists:
            l = []
            for j in i[1]:
                l.append(j.coords)
            Path = NavigationPath(l,i[0],i[0],i[0])
            self.paths.append(Path)

            
            

    def ChangeDestination(self,Destination):
        self.destination = Destination
        #Once the destination changes, the paths in self.paths are no longer applicable, so all NavigationPaths in self.paths are deleted.
        self.paths = []

    def GetPaths(self):
        return self.paths

    def FollowPath(Path,NumInstructions):
        Instructions = Path.GetInstructions()
        for i in range(NumInstructions):
            instruction = Instructions[0]
            #TBA: Actually work with instructions once API is completed


NG = NavigationGraph([0.0,2.0],[0.0,-1.0])
NG.BuildGraph()
NG.Navigate(3)
for i in NG.GetPaths():
    print(i.GetInstructions())
    print(i.GetPathLength())

