#!/usr/bin/env python
# coding: utf-8

# In[1]:


# imports
import numpy as np
import math
import copy


# In[2]:



class NavigationPath:
    def __init__(self, instructions, miles, kilometers, time):
        self.instructions = instructions
        self.miles = miles
        self.kilometers = kilometers
        self.time = time

    def GetInstructions(self):
        return self.instructions

    def GetPathLength(self):
        return (self.miles, self.kilometers, self.time)


class Node:
    def __init__(self, coords):
        # Coords is the primary distinguishing characteristic of a node, no two nodes should have the same lat/long coords.
        # F, G and H are used for the A* search algorithm used in Navigate(n).  G is the weight of all edges taken
        # from the start to the current node, and H is the heuristic (distance between node's coords and destination's coords).
        # Parent is used to trace back the exact path taken from start to goal.
        # Origin is used to find the node in the given graph, as all Nodes created in Navigate(n) are copies of nodes from the graph to ensure the graph is not effected.  However, these copied nodes are obviously not in the graph, so Origin is needed to find the edges of a given node.
        self.coords = coords
        self.f = 0
        self.g = 0
        self.h = 0
        self.parent = []
        self.origin = None

    def UpdateStats(self, g, h):
        self.g = g
        self.h = h
        self.f = g + h

    def GetStats(self):
        return self.f, self.g, self.h

    # EqualCoords checks if the two nodes have identical coords and if so returns true.
    def EqualCoords(self, other):
        if self.coords == other.coords:
            return True
        return False

    # EqualCoordsEqualParent checks if the two nodes have identical coords and the same parent node and if so returns true.
    def EqualCoordsEqualPath(self, other):
        if self.coords == other.coords:
            if len(self.parent) != len(other.parent):
                return False
            for i in range(0, len(self.parent)):
                if self.parent[i].origin != other.parent[i].origin:
                    return False
            return True
        return False

    def UpdateParent(self, parent):
        self.parent = parent

    def GetParent(self):
        return self.parent

    def UpdateOrigin(self, origin):
        self.origin = origin

    def GetOrigin(self):
        return self.origin

    # All nodes in a list are sorted using their F value. (there is not a __eq__ because the dictionary requires that to be left blank for a given node to be a viable key)
    def __lt__(self, other):
        if self.f < other.f:
            return True
        return False

    def __gt__(self, other):
        if self.f > other.f:
            return True
        return False

    def __repr__(self):
        return "%s" % (self.coords)


class Edge:
    # Note: weight might have to be changed to miles and mph down the line once BuildGraph is completed, so that NavigationPath's Miles, Kilometers, and Time variables actually mean something
    def __init__(self, EndNode, weight):
        self.EndNode = EndNode
        self.weight = weight

    def GetEdge(self):
        return self.EndNode, self.weight


class Graph:
    def __init__(self):
        self.graph = {}

    def AddNode(self, Node):
        for i in self.graph:
            if Node.EqualCoords(i):
                return False
        self.graph[Node] = []
        return True

    def AddEdge(self, Node, Edge):
        for i in self.graph[Node]:
            if i.EndNode == Edge.EndNode:
                return False
        NodeValue = self.graph.get(Node)
        NodeValue.append(Edge)
        self.graph[Node] = NodeValue
        return True

    def RemoveEdge(self, Node, Edge):
        self.graph[Node].remove(Edge)
    # Removes the node from the graph and all edges that point to said node.

    def RemoveNode(self, Node):
        del self.graph[Node]
        for i in self.graph:
            for j in self.graph[i]:
                if j.EndNode == Node:
                    self.graph[i].remove(j)

    def getNode(self, location):
        for i in self.graph:
            if i.coords == location.coords:
                return i
        return None

    def ResetGraph(self):
        self.graph = {}


class NavigationGraph:

    def __init__(self, CurrentLocation, Destination):
        self.currentlocation = CurrentLocation
        self.destination = Destination
        self.paths = []
        self.graph = Graph()

        # build graph
        self.BuildGraph()
        return

    def BuildGraph(self):
        # Needs Google Maps API to work
        Brazil = Node([-47.91, -15.76])
        Bolivia = Node([-68.09, -16.46])
        Paraguay = Node([-57.57, -25.29])
        Uruguay = Node([-56.19, -34.87])
        Argentina = Node([-58.41, -34.58])
        Chile = Node([-70.68, -33.44])
        Peru = Node([-77.05, -12.05])
        Ecuador = Node([-78.55, -0.19])
        Colombia = Node([-74.13, 4.72])
        Venezuela = Node([-66.90, 10.51])
        Guyana = Node([-58.19, 6.86])
        Suriname = Node([-55.17, 5.84])
        FrenchGuiana = Node([-52.33, 4.95])

        Nodes = [Brazil, Bolivia, Paraguay, Uruguay, Argentina, Chile, Peru,
                 Ecuador, Colombia, Venezuela, Guyana, Suriname, FrenchGuiana]

        for x in Nodes:
            self.graph.AddNode(x)

        EdgeBrazilFrenchGuiana = Edge(FrenchGuiana, 3293)
        EdgeFrenchGuianaBrazil = Edge(Brazil, 3293)

        EdgeSurinameBrazil = Edge(Brazil, 5275)
        EdgeBrazilSuriname = Edge(Suriname, 5275)

        EdgeGuyanaBrazil = Edge(Brazil, 4826)
        EdgeBrazilGuyana = Edge(Guyana, 4826)

        EdgeBrazilVenezuela = Edge(Venezuela, 5686)
        EdgeVenezuelaBrazil = Edge(Brazil, 5686)

        EdgeColombiaBrazil = Edge(Brazil, 6937)
        EdgeBrazilColombia = Edge(Colombia, 6937)

        EdgeBrazilPeru = Edge(Peru, 4509)
        EdgePeruBrazil = Edge(Brazil, 4509)

        EdgeBrazilBolivia = Edge(Bolivia, 2958)
        EdgeBoliviaBrazil = Edge(Brazil, 2958)

        EdgeBrazilParaguay = Edge(Paraguay, 1849)
        EdgeParaguayBrazil = Edge(Brazil, 1849)

        EdgeBrazilArgentina = Edge(Argentina, 2853)
        EdgeArgentinaBrazil = Edge(Brazil, 2853)

        EdgeBrazilUruguay = Edge(Uruguay, 2834)
        EdgeUruguayBrazil = Edge(Brazil, 2834)

        EdgeFrenchGuianaSuriname = Edge(Suriname, 408)
        EdgeSurinameFrenchGuiana = Edge(FrenchGuiana, 408)

        EdgeSurinameGuyana = Edge(Guyana, 450)
        EdgeGuyanaSuriname = Edge(Suriname, 450)

        EdgeGuyanaVenezuela = Edge(Venezuela, 2164)
        EdgeVenezuelaGuyana = Edge(Guyana, 2164)

        EdgeVenezuelaColombia = Edge(Colombia, 1429)
        EdgeColombiaVenezuela = Edge(Venezuela, 1429)

        EdgeColombiaPeru = Edge(Peru, 2857)
        EdgePeruColombia = Edge(Colombia, 2857)

        EdgeColombiaEcuador = Edge(Ecuador, 1099)
        EdgeEcuadorColombia = Edge(Colombia, 1099)

        EdgeEcuadorPeru = Edge(Peru, 1783)
        EdgePeruEcuador = Edge(Ecuador, 1783)

        EdgePeruBolivia = Edge(Bolivia, 1510)
        EdgeBoliviaPeru = Edge(Peru, 1510)

        EdgeBoliviaArgentina = Edge(Argentina, 3922)
        EdgeArgentinaBolivia = Edge(Bolivia, 3922)

        EdgeBoliviaParaguay = Edge(Paraguay, 3922)
        EdgeParaguayBolivia = Edge(Bolivia, 3922)

        EdgeParaguayArgentina = Edge(Argentina, 1237)
        EdgeArgentinaParaguay = Edge(Paraguay, 1237)

        EdgeArgentinaUruguay = Edge(Uruguay, 235)
        EdgeUruguayArgentina = Edge(Argentina, 235)

        EdgeArgentinaChile = Edge(Chile, 1400)
        EdgeChileArgentina = Edge(Argentina, 1400)

        self.graph.AddEdge(Brazil, EdgeBrazilPeru)
        self.graph.AddEdge(Peru, EdgePeruBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilFrenchGuiana)
        self.graph.AddEdge(FrenchGuiana, EdgeFrenchGuianaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilSuriname)
        self.graph.AddEdge(Suriname, EdgeSurinameBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilGuyana)
        self.graph.AddEdge(Guyana, EdgeGuyanaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilVenezuela)
        self.graph.AddEdge(Venezuela, EdgeVenezuelaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilColombia)
        self.graph.AddEdge(Colombia, EdgeColombiaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilBolivia)
        self.graph.AddEdge(Bolivia, EdgeBoliviaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilParaguay)
        self.graph.AddEdge(Paraguay, EdgeParaguayBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilArgentina)
        self.graph.AddEdge(Argentina, EdgeArgentinaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilUruguay)
        self.graph.AddEdge(Uruguay, EdgeUruguayBrazil)

        self.graph.AddEdge(FrenchGuiana, EdgeFrenchGuianaSuriname)
        self.graph.AddEdge(Suriname, EdgeSurinameFrenchGuiana)

        self.graph.AddEdge(Suriname, EdgeSurinameGuyana)
        self.graph.AddEdge(Guyana, EdgeGuyanaSuriname)

        self.graph.AddEdge(Guyana, EdgeGuyanaVenezuela)
        self.graph.AddEdge(Venezuela, EdgeVenezuelaGuyana)

        self.graph.AddEdge(Venezuela, EdgeVenezuelaColombia)
        self.graph.AddEdge(Colombia, EdgeColombiaVenezuela)

        self.graph.AddEdge(Colombia, EdgeColombiaPeru)
        self.graph.AddEdge(Peru, EdgePeruColombia)

        self.graph.AddEdge(Colombia, EdgeColombiaEcuador)
        self.graph.AddEdge(Ecuador, EdgeEcuadorColombia)

        self.graph.AddEdge(Peru, EdgePeruEcuador)
        self.graph.AddEdge(Ecuador, EdgeEcuadorPeru)

        self.graph.AddEdge(Peru, EdgePeruBolivia)
        self.graph.AddEdge(Bolivia, EdgeBoliviaPeru)

        self.graph.AddEdge(Bolivia, EdgeBoliviaArgentina)
        self.graph.AddEdge(Argentina, EdgeArgentinaBolivia)

        self.graph.AddEdge(Bolivia, EdgeBoliviaParaguay)
        self.graph.AddEdge(Paraguay, EdgeParaguayBolivia)

        self.graph.AddEdge(Argentina, EdgeArgentinaChile)
        self.graph.AddEdge(Chile, EdgeChileArgentina)

        self.graph.AddEdge(Argentina, EdgeArgentinaParaguay)
        self.graph.AddEdge(Paraguay, EdgeParaguayArgentina)

        self.graph.AddEdge(Argentina, EdgeArgentinaUruguay)
        self.graph.AddEdge(Uruguay, EdgeUruguayArgentina)

    # Returns the Elucidian Distance between the Node and NavigationGraph's destination variable.
    def Heuristic(self, Node):
        return math.sqrt((Node.coords[0]-self.destination.coords[0])**2 + (Node.coords[1]-self.destination.coords[1])**2)

    def Navigate(self, paths):
        # Instantiates goal and start nodes based on the destination and current location, respectively
        Goal = copy.deepcopy(self.graph.getNode(self.destination))
        Goal.UpdateOrigin(self.graph.getNode(self.destination))
        Start = copy.deepcopy(self.graph.getNode(self.currentlocation))
        Start.UpdateOrigin(self.graph.getNode(self.currentlocation))
        Start.UpdateParent([])
        # Sets up the loop
        OpenList = []
        ClosedList = []
        ParentLists = []
        counter = 0
        OpenList.append(Start)
        while counter < paths:
            # If at any point the OpenList has no Nodes left, the while loop breaks.  This is to make sure if the
            # of unique paths to the goal are less than the variable paths, Navigate doesn't break
            if len(OpenList) == 0:
                break
            # Sorts the list, takes the first Node from the list and deletes said node from OpenList
            OpenList.sort()
            while OpenList[0] == None:
                del OpenList[0]
            CurrentNode = OpenList[0]
            del OpenList[0]
            # Adds deleted node to the closed list, and gets the F, G and H values from the current Node
            ClosedList.append(CurrentNode)
            CurrentF, CurrentG, CurrentH = CurrentNode.GetStats()
            CurrentParent = copy.deepcopy(CurrentNode.GetParent())
            CurrentParent.append(CurrentNode)
            # print(CurrentParent)

            # If the given node is Goal and the number of paths already found are less than the given variable paths, the entire path of nodes is added to ParentList, which in turn is added to ParentLists.
            if CurrentNode.EqualCoords(Goal):
                if counter < paths:
                    ParentLists.append((CurrentG, CurrentParent))
                    counter += 1
                    # print(counter)
            else:
                # For every unique edge CurrentNode has, it checks to see if the Node said edge has is already in OpenList or ClosedList and comes from the same parent/edge.
                for i in self.graph.graph[CurrentNode.GetOrigin()]:
                    ListCheck = True
                    E = copy.deepcopy(i.EndNode)
                    E.UpdateParent(CurrentParent)
                    E.UpdateOrigin(i.EndNode)
                    E.UpdateStats(CurrentG+i.weight, self.Heuristic(i.EndNode))
                    # If there is already a child node from that edge in ClosedList, nothing further is done.
                    coordcheck = []
                    for i in E.parent:
                        if i.coords in coordcheck:
                            ListCheck = False
                        coordcheck.append(i.coords)
                    for j in ClosedList:
                        if E.EqualCoordsEqualPath(j):
                            ListCheck = False
                            # print("e")
                            # print(E.parent)
                            # print(j.parent)
                            # print()
                    # If there is already a child node from that edge in OpenList, the node in OpenList is updated to be either its old g value, or the g value obtained currently, whichever is less.
                    # This ensures the node in question will always exemplify the fastest path to a given edge.
                    if j in OpenList:
                        if E.EqualCoordsEqualPath(j):
                            ListCheck = False
                            # print("f")
                            # print(E.parent)
                            # print(j.parent)
                            # print()
                            i.UpdateStats(min(j.g, E.g), j.h)
                    # If the given node from the edge is in neither OpenList or ClosedList, the node is appended to OpenList.
                    if ListCheck == True:
                        OpenList.append(E)

                        # print(CurrentG)
                        # print(CurrentNode.coords,E.coords,E.g)
        # Once the number of unique paths to the destination or the fastest paths number of unique paths is found,
        # whichever is less,
        # these paths stored in ParentLists are converted to NavigationPaths and appended to self.paths.
        # Before this, self.paths is set to no paths, so that the unique paths in Navigate() do not stack.
        self.paths = []
        for i in ParentLists:
            l = []
            for j in i[1]:
                l.append(j.coords)
            Path = NavigationPath(l, i[0], i[0], i[0])
            self.paths.append(Path)

    def ChangeDestination(self, Destination):
        self.destination = Destination
        # Once the destination changes, the paths in self.paths are no longer applicable, so all NavigationPaths in self.paths are deleted.
        self.paths = []

    def GetPaths(self):
        return self.paths

    def FollowPath(Path, NumInstructions):
        Instructions = Path.GetInstructions()
        for i in range(NumInstructions):
            instruction = Instructions[0]
            # TBA: Actually work with instructions once API is completed


# In[3]:


dict_countries = dict(
    Brazil = Node([-47.91, -15.76]),
    Bolivia = Node([-68.09, -16.46]),
    Paraguay = Node([-57.57, -25.29]),
    Uruguay = Node([-56.19, -34.87]),
    Argentina = Node([-58.41, -34.58]),
    Chile = Node([-70.68, -33.44]),
    Peru = Node([-77.05, -12.05]),
    Ecuador = Node([-78.55, -0.19]),
    Colombia = Node([-74.13, 4.72]),
    Venezuela = Node([-66.90, 10.51]),
    Guyana = Node([-58.19, 6.86]),
    Suriname = Node([-55.17, 5.84]),
    FrenchGuiana = Node([-52.33, 4.95]),
)


dict_coords = {
"[-47.91, -15.76]": "Brazil",
"[-68.09, -16.46]": "Bolivia",
"[-57.57, -25.29]": "Paraguay",
"[-56.19, -34.87]": "Uruguay",
"[-58.41, -34.58]": "Argentina",
"[-70.68, -33.44]": "Chile",
"[-77.05, -12.05]": "Peru",
"[-78.55, -0.19]": "Ecuador",
"[-74.13, 4.72]": "Colombia",
"[-66.9, 10.51]": "Venezuela",
"[-58.19, 6.86]": "Guyana",
"[-55.17, 5.84]": "Suriname",
"[-52.33, 4.95]": "FrenchGuiana",
}


# In[4]:


# UserInterface class
class UserInterface:
    def __init__(self):
        # where to store current location and destination
        self.countries = {
            "current location": None,
            "destination": None
        }

        self.nav_graph = None
        self.list_path = None
        return

    def launch_Interface(self):
        # get user params
        self.input_valid_country("current location")
        self.input_valid_country("destination")

        # break line
        print("")

        # init graph
        self.nav_graph = NavigationGraph(
            CurrentLocation=self.countries["current location"],
            Destination=self.countries["destination"]
        )

        # find all paths
        self.nav_graph.Navigate(10)
        self.list_path = self.nav_graph.GetPaths()

        # find shortest path
        # index_shortest_path = np.argmin([path.GetPathLength()[0] for path in self.list_path])

        # sort path according to length
        self.list_path.sort(key=lambda path: path.GetPathLength()[0])

        # show shortest path
        name_current_location = dict_coords[str(self.countries['current location'])]
        name_destination = dict_coords[str(self.countries['destination'])]

        print(f"Shortest path from {name_current_location} to {name_destination}")
        UserInterface.show_path(self.list_path[0], 0)
        
        # break line
        print("")

        # show other paths
        print("The other available paths")
        for index_path, path in enumerate(self.list_path[1:]):
            UserInterface.show_path(path, index_path + 1)

        # choose path
        index_choosen_path = self.input_path_index()

        # break line
        print("")

        # re print path
        UserInterface.show_path(self.list_path[index_choosen_path], index_choosen_path)
        print("You reached your destination")
        return

    def input_valid_country(self, type_country):
        prompted_message = f"Choose {type_country}: "
        is_valid_country = False

        list_valid_countries = "\n".join(list(dict_countries.keys()))

        while not is_valid_country:
            try:
                input_country = input(prompted_message)
                # correct form
                input_country = input_country.lower().capitalize()
                self.countries[type_country] = dict_countries[input_country]
                # break loop
                is_valid_country = True
                
                # print the selected country
                # print(f"{type_country.capitalize()}: {input_country.capitalize()}")
            except:
                prompted_message = """Incorrect country! Please select a valid country name:\n%s
                """ % list_valid_countries
                continue

        return

    def input_path_index(self):
        prompted_message = "Select path: (number from 0 to 9)"

        while True:
            inputed_number = input(prompted_message)

            if not inputed_number.isnumeric():
                prompted_message = "Please enter a number not characters"
                print(inputed_number)
                continue
                
            inputed_number = int(inputed_number) 

            if  inputed_number >= 10 or inputed_number < 0:
                prompted_message = "Please enter a number between 0 and 9"
                print(inputed_number)
                continue

            break

        return inputed_number

    @staticmethod
    def show_path(path, index_path):
        # replace coords with countries' names
        path_as_str = str(index_path) + ") " + " --> ".join([dict_coords[str(n)] for n in path.GetInstructions()])
        print(path_as_str)
        return 


# In[5]:


# init UI
ui = UserInterface()

# launch UI and follow the prompt
ui.launch_Interface()

