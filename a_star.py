import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

#Node class for A* pathfinding
class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    #Use to compare nodes
    def __eq__(self, other):
        return self.position == other.position #Nodes are equal if their position in the maze is the equal

#This function return the path of the search
def return_path(current_node,maze):
    # Initialize an empty path list
    path = []
    
    # Get the shape of the maze
    no_rows, no_columns = np.shape(maze)
    
    # Initialize the result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    
    # Set the current node
    current = current_node
    
    # Traverse the path from the current node to the root (start node)
    while current is not None:
        # Append the current node's position to the path list
        path.append(current.position)
        
        # Move to the parent node
        current = current.parent
    
    # Reverse the path list to get the path from start to end
    path = path[::-1]
    
    # Initialize the start value
    start_value = 0
    
    # Update the result maze with the path found by A-star search
    # Each step in the path is incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    # Return the path
    return path

def manhattan_cost(node: Node, end_node: Node) -> float:
    # The Manhattan cost is the sum of the absolute differences of the Cartesian coordinates
    # It represents the minimum cost to reach the end node from the current node when only orthogonal (vertical and horizontal) movements are allowed
    return abs(node.position[0]-end_node.position[0])+abs(node.position[1]-end_node.position[1])

def euclidian_cost(node: Node, end_node: Node) -> float:
    # The Euclidean cost is the square root of the sum of the squares of the differences of the Cartesian coordinates
    return sqrt((node.position[0]-end_node.position[0])**2+(node.position[1]-end_node.position[1])**2)

def search(maze, start, end, heuristic_type = 'euclidean'):

    print("searching ....")

    # Transpose the maze to align with the coordinate system
    maze = maze.T

    # Create start and end node with initialized values for g, h and f
    start_node = Node(parent = None, position=start)
    start_node.g = 0
    start_node.h = 0
    start_node.f = start_node.g + start_node.h

    end_node = Node(parent=None, position=end)
    end_node.g = 0
    end_node.h = 0
    end_node.f = end_node.g + end_node.h

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration. 
    # From here we will find the lowest cost node to expand next
    yet_to_visit_list = []  
    # in this list we will put all node those already explored so that we don't explore it again
    visited_list = [] 

    # Add the start node
    yet_to_visit_list.append(start_node)

    # Adding a stop condition to avoid any infinite loop
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10
    # TODO PART 4 what squares do we search . serarch movement is left-right-top-bottom 
    #(4 movements) from every positon
    move  =  [[-1, 0 ], # go up
              [ 0, -1], # go left
              [ 1, 0 ], # go down
              [ 0, 1 ], # go right
              [-1, -1], # go up left
              [ 1, -1], # go down left
              [-1, 1 ], # go up right
              [ 1, 1 ]] # go down right

    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit list and add this node to visited list
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited list then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit list then ignore it
                d) else move the child to yet_to_visit list
    """
    # Get the size of the maze
    no_rows, no_columns = np.shape(maze)

    # Loop until you find the end
    while len(yet_to_visit_list) > 0:
        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1    

        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        # If max iterations reached, return the path
        #such as it may be no solution or computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            return return_path(current_node,maze)

        # Pop current node out off yet_to_visit list, add to visited list
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # If goal is reached, return the path
        if current_node == end_node:
            return return_path(current_node,maze)

        # Generate children from all adjacent squares
        children: list[Node] = []

        for new_position in move: 

            # TODO PART 4 Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # TODO PART 4 Make sure within range (check if within maze boundary)
            if (node_position[0] > (no_rows-1) or 
                node_position[0] < 0 or 
                node_position[1] > (no_columns-1) or 
                node_position[1] < 0):
                continue

            # Ignore nodes that are obstacles
            if maze[node_position[0],node_position[1]] > 0.8:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # TODO PART 4 If child is on the visited list, ignore it
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # TODO PART 4 Create the f, g, and h values
            child.g = child.parent.g + 1
            
            # Heuristic costs calculated here
            if heuristic_type == "euclidean":
                child.h = euclidian_cost(child, end_node)
            else:
                child.h = manhattan_cost(child, end_node)

            child.f = child.g + child.h

            # If child is already in the yet_to_visit list and g cost is already lower, ignore it
            if len([i for i in yet_to_visit_list if child == i and child.g >= i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)
