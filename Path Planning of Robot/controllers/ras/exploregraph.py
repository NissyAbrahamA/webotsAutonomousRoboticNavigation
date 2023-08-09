import math
import networkx as nx

#The code was run in explore mode multiple times and graphs were merged as exploration wouldnt be complete within time to live
class ExploreGraph:
    """
    A class for exploring the graph by taking alternate turns at junctions and create a graph for the 
    car to follow when battery is low.
    
    While exploring the graph, we look for the current position of the graph and identify if the node has been 
    ecplored already. Based on it, the path for further exploration is chosen. (similar to SLAM RAS unit 12)
    """
    def explore_graph(self, lastnode, car_md):
        """
        Explores the environment to create a graph and the generated graph owuld be stored in a .graphml to be used to 
        identify the shortest path when battery is low
        
        Inputs:
            lastnode : a tuple representing the last node in the graph that was explored
            car_md :  a string representing the current direction of the car
        Returns:
            Turn_angle : a float representing the steering angle that needs to be taken when in junction that was not taken 
            earlier in the exploration. Idnetified by looking for the node following the last instance of the current node
            and comapring the node values to get the necessary turn angle
        """
        
        #Since the graph may not have the exact node (as gps cordinates are stored with 4 decimal places in the graph), identify the
        #closest node by distance to the current node
        close_node_index = self.check_node_in_graph(lastnode)
        turn_angle = 0
        #print(close_node_index)
        if close_node_index != -1:
            #If the node is already explored, the identify the 50th node following the current node for comparison
            node_list = list(self.G.nodes())
            close_node = node_list[close_node_index]
            next_node_index = close_node_index + 50
            
            #If 50th node index exists (i.e the current node is not just added), then get the 50th node for comarison
            if next_node_index < len(self.G.nodes()):
                next_node = node_list[next_node_index]
                # print('close_node')
                # print(close_node)
                # print('next_node')
                # print(next_node)
                
                #This code is to take the direction opposite to what was taken earlier to explore
                if car_md == 'X':
                    if close_node[0] < next_node[0] and close_node[1] < next_node[1]:
                        turn_angle = self.LeftTurn
                    elif close_node[0] > next_node[0] and close_node[1] > next_node[1]:
                        turn_angle = self.LeftTurn
                    elif close_node[0] < next_node[0] and close_node[1] > next_node[1]:
                        turn_angle = self.RightTurn
                    elif close_node[0] > next_node[0] and close_node[1] < next_node[1]:
                        turn_angle = self.RightTurn   
                        
                if car_md == 'Y':
                    if close_node[0] < next_node[0] and close_node[1] < next_node[1]:
                        turn_angle = self.RightTurn
                    elif close_node[0] > next_node[0] and close_node[1] > next_node[1]:
                        turn_angle = self.RightTurn
                    elif close_node[0] < next_node[0] and close_node[1] > next_node[1]:
                        turn_angle = self.LeftTurn
                    elif close_node[0] > next_node[0] and close_node[1] < next_node[1]:
                        turn_angle = self.LeftTurn 
			
        return turn_angle
        
    def check_node_in_graph(self,last_node):
        """
        Calculates the distance between current node to all the nodes in the graph to identify the node closest to the current node
        Input: lastnode : a tuple representing the last node in the graph that was explored
        Returns: a integer that is the index of the identified closest node in graph, else -1
        """
        for i, node in enumerate(self.G.nodes()):
            dist = math.sqrt((node[0]-last_node[0])**2 + (node[1]-last_node[1])**2)
            if dist < 1:
                return i
        return -1 
    