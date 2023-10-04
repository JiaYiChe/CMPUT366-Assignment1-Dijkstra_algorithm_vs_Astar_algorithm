import heapq

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._h = 0
        self._cost = 0
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def get_h(self):
        """
        Returns the h-value of the state
        """
        return self._h
    
    def get_cost(self):
        """
        Returns the cost of the state (g for Dijkstra's and f for A*)
        """
        return self._cost
    
    def set_g(self, cost):
        """
        Sets the g-value of the state
        """
        self._g = cost
    
    def set_h(self, h):
        """
        Sets the h-value of the state
        """
        self._h = h
    
    def set_cost(self, cost):
        """
        Sets the cost of a state (g for Dijkstra's and f for A*)
        """
        self._cost = cost

class Search:
    """
    Interface for a search algorithm. It contains an OPEN list and a CLOSED list.

    The OPEN list is implemented with a heap, which can be done with the library heapq
    (https://docs.python.org/3/library/heapq.html).    
    
    The CLOSED list is implemented as a dictionary where the state hash value is used as key.
    """
    def __init__(self, gridded_map):
        self.map = gridded_map
        self.OPEN = []
        self.CLOSED = {}
    
    def search(self, start, goal):
        """
        Search method that needs to be implemented (either Dijkstra or A*).
        """
        raise NotImplementedError()
            
class Dijkstra(Search):
            
    def search(self, start, goal):
        """
        Disjkstra's Algorithm: receives a start state and a goal state as input. It returns the
        cost of a path between start and goal and the number of nodes expanded.

        If a solution isn't found, it returns -1 for the cost.
        """
        #empty open and close to avoid future mix
        self.OPEN=[]
        self.CLOSED={}
        #give initial node to open and close
        heapq.heappush(self.OPEN,start)
        self.CLOSED[start.state_hash()] = start.get_g()
        #track on number of node expanded
        node_expanded = 0
        #check if open list empty
        while len(self.OPEN)>0:
            node = heapq.heappop(self.OPEN)
            node_expanded+=1
            #if current node is goal then return path cost and number of node expanded 
            if node.__eq__(goal):
                path_cost = self.CLOSED[node.state_hash()]
                return path_cost, node_expanded
            #get children of current node
            children = self.map.successors(node)
            #read current node's child one by one
            for child in children:
                #if child node not in close dict then save it
                if child.state_hash() not in self.CLOSED:
                    child.set_cost(child.get_g())
                    heapq.heappush(self.OPEN,child)
                    self.CLOSED[child.state_hash()] = child.get_g()

                #if child node in close dict and this new child node cost less than the previous path, update the cost
                if child.state_hash() in self.CLOSED and child.get_g() < self.CLOSED[child.state_hash()]:
                    '''
                    counter=0
                    
                    for i in self.OPEN:
                        if node.__eq__(child):
                            child.set_cost(child.get_g())
                            heappop(self.OPEN[counter])
                            heapq.heappush(self.OPEN,child)
                            break;
                        counter+=1
                    '''
                    child.set_cost(child.get_g())
                    self.CLOSED[child.state_hash()] = child.get_g()
                    #reorder the open list after change node's value
                    heapq.heapify(self.OPEN)

        return -1, 0
    
class AStar(Search):
    
    def h_value(self, state, goal):
        #get absolute value of x and y from current node to goal
        absX = abs(state.get_x() - goal.get_x())
        absY = abs(state.get_y() - goal.get_y())
        h = max(absX, absY) + 0.5 * min(absX, absY)
        return h
            
    def search(self, start, goal):
        """
        A* Algorithm: receives a start state and a goal state as input. It returns the
        cost of a path between start and goal and the number of nodes expanded.

        If a solution isn't found, it returns -1 for the cost.
        """
        #empty open and close to avoid future mix
        self.OPEN=[]
        self.CLOSED={}
        #for A* only save the node to open list at beginning
        heapq.heappush(self.OPEN,start)
        #track on the number of node expanded
        node_expanded = 0
        #check if open list still have nodes
        while len(self.OPEN)>0:
            node = heapq.heappop(self.OPEN)
            node_expanded+=1
            #if current node is goal then return the path cost and number of node expanded
            if node.__eq__(goal):
                return node.get_g(), node_expanded
            #save the node to close dict after pop out from open list
            self.CLOSED[node.state_hash()] = node
            #get children list of current node
            children = self.map.successors(node)
            #check every node in children list
            for child in children:
                #calculate the cost from current node to goal
                h = self.h_value(child, goal)
                #if child node already in closed dict then do nothing
                if child.state_hash() in self.CLOSED:
                    continue
                #if child node is in open list we check if this path cost less than old path, if so update it in open list
                if child in self.OPEN:
                    '''
                    counter=0
                    for i in self.OPEN:
                        if i.__eq__(child):
                            break;
                        counter+=1
                    '''
                    oldNode = self.OPEN.index(child)
                    old = self.OPEN[oldNode].get_cost()
                    new = child.get_g()+h
                    

                    if  new<old :
                        #child.set_h(h)
                        #child.set_cost(new)
                        #child.set_g(child.get_g())
                        #self.OPEN[counter] = child
                        #self.OPEN[counter].set_h(h)
                        self.OPEN[oldNode].set_g(child.get_g())
                        self.OPEN[oldNode].set_cost(new)
                        heapq.heapify(self.OPEN)
                #if the child node not in either open or close then add it into open list
                else:
                    g = child.get_g()
                    #child.set_h(h)
                    child.set_cost(g+h)
                    child.set_g(g)
                    heapq.heappush(self.OPEN,child)

        return -1, 0
    
