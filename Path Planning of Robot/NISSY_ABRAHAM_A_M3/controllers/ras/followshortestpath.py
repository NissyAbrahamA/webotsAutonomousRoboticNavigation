import math
import networkx as nx

class FollowShortestPath:
    """
    A class for following the shortest path to reach the destination that is identified with the loaded graph. 
    The turns at each intersection is decided and the path is followed to recharge the car
    
    Since the car is Ackerman drive,  it is suited for easier manipulation with any of the angles identified 
    below, allowing for smooth and precise turns by controlling the rotation of individual wheels.
    (RAS Unit 7 Locomotion)
    """
    
    def follow_path_to_charging(self, lastnode, car_md):
        """
        provides the steering angle to follow the path that was identified as the shortest path to reach destination
        from the currrent node of the car
        
        Inputs:
            lastnode : a tuple representing the last node in the graph that was explored
            car_md :  a string representing the current direction of the car
        Returns:
            Turn_angle : a float representing the steering angle that needs to be taken when in junction 
            that was identified from the shortest path from graph.Identified by looking for the node following 
            the current node and comparing the node values to get the necessary turn angle
            
        """
        next_node = None
        #print(lastnode)
        #print(car_md)
        turn_angle = 0
        src_node = tuple(map(int, map(round, lastnode)))
        
        #If the current node is not available in the graph, then identify the closest node in the shortest path
        #to the current node
        if src_node not in self.Path:
            #print('not in path')
            src_node, src_index =self.check_node_inpath(self.Path, src_node)
            #print('retunedsrc'+str(src_node))
        else:
            src_index = self.Path.index(src_node)
        #Identify the next node for comparison from the shortest path identified. If there is no 10th node
        #then compare with the destination node
        #print(src_node) 
        next_index = src_index + 10
        if next_index < len(self.Path):
            next_node = self.Path[next_index]
        else:
            next_node = self.Path[-1]
        #print(next_node)
        
        #This code is to identify the steering angle to follow the shortest path. 
        if src_node != None: 
            #Turn in the direction of the next node to follow path
            src_node = tuple(map(int, src_node.strip('()').split(', ')))
            next_node = tuple(map(int, next_node.strip('()').split(', ')))
            # print(src_node[0])
            # print(next_node[0])
            # print(src_node[1])
            # print(next_node[1])
            if car_md == 'X':
                if src_node[0] < next_node[0] and src_node[1] < next_node[1]:
                    turn_angle = self.RightTurn
                elif src_node[0] > next_node[0] and src_node[1] > next_node[1]:
                    turn_angle = self.RightTurn
                elif src_node[0] < next_node[0] and src_node[1] > next_node[1]:
                    #print('3')
                    turn_angle = self.LeftTurn
                elif src_node[0] > next_node[0] and src_node[1] < next_node[1]:
                    turn_angle = self.LeftTurn
                else:
                    turn_angle = 0
                    
            if car_md == 'Y':
                if src_node[0] < next_node[0] and src_node[1] < next_node[1]:
                    turn_angle = self.LeftTurn
                elif src_node[0] > next_node[0] and src_node[1] > next_node[1]:
                    turn_angle = self.LeftTurn
                elif src_node[0] < next_node[0] and src_node[1] > next_node[1]:
                    turn_angle = self.RightTurn
                elif src_node[0] > next_node[0] and src_node[1] < next_node[1]:
                    turn_angle = self.RightTurn 
                else:
                    turn_angle = 0
        else:
            turn_angle = 0
            
        return turn_angle
        
         
            
    def check_node_inpath(self, nodes, current_node):
        """
        Calculates the distance between current node to all the nodes in the shortest path to 
        identify the node closest to the current node in the shortest path
        Input:  nodes: a list of tuples representing the nodes in the shortest path
                current_node : a tuple representing the last node in the graph that was explored
        Returns: nearest_node_index : a integer that is the index of the identified closest node in graph
                 nearest_node: a tuple of nearestest node to the current node
        
        """
        nearest_node = None
        min_distance = float('inf')
        nearest_node_index = -1
    
        current_x, current_y = current_node
        #print('curr'+str(current_node))
        for i, node in enumerate(nodes):
            node_x, node_y = map(int, node.strip('()').split(','))
            dist = math.sqrt((node_x - current_x) ** 2 + (node_y - current_y) ** 2)
            if dist < 5 and dist < min_distance:
                nearest_node = node
                min_distance = dist
                nearest_node_index = i
        #print('near' + str(nearest_node))
        return nearest_node, nearest_node_index