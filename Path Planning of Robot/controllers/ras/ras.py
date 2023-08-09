from rasrobot import RASRobot
from roaddetector import RoadDetector
from exploregraph import ExploreGraph
from followshortestpath import FollowShortestPath
import numpy as np
import time
import cv2
import torch
import math
import random
import networkx as nx
from math import sqrt
import configparser
import ast
import re


""" Mission 3: Keeping the battery charged while navigating 
 The Navigation robotic subsystem(RAS unit-3) is a component of autonomous robotic systems that enables
 them to move and navigate in the environment. It uses various sensors, such as GPS and cameras (RAS unit-8), 
 to perceive the surroundings, and algorithms to plan and execute a path to reach a goal location
 In this exercise, an autonomous car with ackerman drive(RAS unit 7) is used. The car is provided with RGB camera and 
 GPS sensor. Based on the graph explored already, path to the destination is planned with Dijkstra's 
 shortest path algorithm(networkx- Network to create a graph for path planing RAS unit 11) to reach the destination.

The agent plays both deliberative and reactive role in this mission. There are two modes of execution in this task. First,
as a deliberative agent, explore the graph(ExploreMode = True). The code was executed in ExploreMode and the graph is 
available for the robot to run in the actual mode.
In the actual execution, the agent is a reactive navigation agent. The robot drives around randomly based on the input image,
taking right and left turns randomly at intersections. When battery is low, identifies the shortest path to go to the 
destination to recharge. Here both image and GPS sensors are used to sense the environment and act accordingly.
Therefore it follows PLAN; SENSE-ACT approach. (RAS Unit 3  Software organisation)

"""
class MyRobot(RASRobot,RoadDetector,ExploreGraph,FollowShortestPath):

    def __init__(self):
        super(MyRobot, self).__init__()
        # Initialise and resize a new window
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("output", 128*4, 64*4)

        #Initialise the variables        
        self.Turn = False
        self.ContourFound = 0
        self.DimCamera = False
        self.counter = 0
        self.RightTurn = 0.2
        self.LeftTurn = -0.2
        self.TurnSpeed = 20
        self.NormalSpeed = 40
        self.RightTurnAdjusted = 0.4
        self.random_choice = None
        self.turn_steering_angle = 0
        self.steering_angle = 0
        self.G = nx.DiGraph()
        self.ExploreMode = False
        self.Path = None
     
    
    def set_random_turn_angle(self):
        """
        Randomly choose right/left turns for the car to take in intersections
        
        Sets the turn_steering_angle randomly right if random integer selected is greater than 5 else left
        """
        if self.random_choice is None:
            self.random_choice = random.randint(0, 10)
        self.turn_steering_angle = self.RightTurn if self.random_choice > 5 else self.LeftTurn

    def calculate_steering_angle(self, mask_yellow, image, lastnode,car_md):
        """
        Calculates the steering angle for the car to take in intersections. If there is yellow contours found 
        in the masked image returned by detect_road, then calculate the centroid of the yellow pixels and set 
        the steering angle with respect to it. If no yellow contours are found, i.e arrived at a intersection
        where there is no yellow line, then look for mode of execution. If ExploreMode, then call the explore_graph
        to check if the node was explored already and determine the turn steering angle, if not explored, the  randomly 
        take turns.  If not in Explore mode, the check if the shortest path to destination exist, i.e Battery is
        Low if shortest path is avialable. If exists, follow the path to charging and determine the steering angle
        else randomly take turns and drive around
        
        Inputs:
            mask_yellow - Yellow Masked image returned by the detect road function
            image -  actual image observed by the camera
            lastnode - a tuple representing the current node/closest to current node in the graph
            car_md - a string representing the current direction of the car
            
        Returns:
            steering_angle - a float that is used to set the steering angle of the car
        """
        height, width, _ = image.shape
        moments = cv2.moments(mask_yellow)
        #print('sm' + str(moments["m00"]))
        if moments["m00"] > 0 :
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.circle(image, (cx, cy), 10, (0, 255, 255), -1)
            self.steering_angle = (cx - width/2) / (width/2)
            #print('strandle' +str(self.steering_angle))
            self.ContourFound = self.ContourFound + 1
            
            if self.Turn == True and self.ContourFound < 100:
                self.DimCamera = True
            else:
                self.DimCamera = False
                self.ContourFound = 0
                self.Turn = False
        else:
            #Expllore mode execution
            if self.ExploreMode:
                turn_angle = self.explore_graph(lastnode, car_md)
                #print(turn_angle)
                if turn_angle != 0:
                    self.turn_steering_angle = turn_angle
                else:
                    self.set_random_turn_angle()
            else:
                #Actual execution
                if self.Path == None:
                    #Ramdomly drive arond until battery is low
                    self.set_random_turn_angle()
                else:
                    #When battery is low, follow the shortest path identified
                    path_angle = self.follow_path_to_charging(lastnode, car_md)
                    self.turn_steering_angle = path_angle
                    
            #This code is to ensure that the angle is set till the end of the turn    
            if self.Turn == False:
                self.steering_angle = self.turn_steering_angle
            else:
                self.steering_angle = self.steering_angle
            self.Turn = True
            #print(self.steering_angle)
            #print('3-self.ContourFound-'+str(self.ContourFound)+'self.DimCamera-'+str(self.DimCamera)+'self.Turn-'+str(self.Turn))
        #print( str(self.ContourFound) + str(self.random_choice))
        if self.random_choice is not None and self.ContourFound == 99:
            self.random_choice = None
        
        return self.steering_angle
    
    def get_car_direction(self, last_node, prev_node):
        """Determine the direction of the car based on the previous and current GPS coordinates
        Input:
            last_node - a tuple representing the current node/closest to current node in the graph
            
        Returns:
            car_md - a string indicating the direction of the movement of the car 
        """
        x_diff = int(last_node[0]) - int(prev_node[0])
        y_diff = int(last_node[1]) - int(prev_node[1])
        if abs(x_diff) <= abs(y_diff):
            return 'X'
        else:
            return 'Y'

    
    def creategraph(self):
        """creates a graph with all the nodes and edges added to the graph in explore mode
        Returns:
            graph with name rasgraph.graphml is stored in the source folder
        """
        for n in self.G.nodes:
            attrs = self.G.nodes[n]
            for attr in list(attrs):
                if isinstance(attrs[attr], tuple):
                    attrs[attr] = str(attrs[attr])
        nx.write_graphml(self.G, "rasgraph.graphml")       
        
    def run(self):
        """
        This function implements the main loop of the robot.
        """
        #variable initialisation
        low_battery = 0.40*self.time_to_live #Assuming low battery as 40% 
        destination = "(-210, -165)"
        car_md = None 
        prev_gps_cords = None
        highbattery = 0.9*self.time_to_live
        dis_destination=10000 #default    
        
        #If the code is not executed in explore mode, then load the graph to find the shortest path when battery is low.
        if self.ExploreMode == False:
            graph = nx.read_graphml("rasgraph.graphml")
            #modified the file and saved with this conversion  as it takes time for the conversion
            # pattern = re.compile(r"[-+]?\d*\.\d+|[-+]?\d+") 

            # mapping = {}
            # for n in graph.nodes():
                # nums = pattern.findall(n)
                # nums = [int(float(num)) for num in nums] 
                # mapping[n] = tuple(nums)
            # graph = nx.relabel_nodes(graph, mapping)
          

        while self.tick():
            #get camera image for rasrobot
            image = self.get_camera_image()
            #get gps values from rasrobot
            gps_values = self.get_gps_values()
            
            #Gps cordinates are added as nodes and the neighbouring nodes are added as edges in the below code
            lat = round(gps_values[0],4)
            long = round(gps_values[1],4)
            gps_cords = (lat, long)
            self.G.add_node(gps_cords, pos=gps_cords)
            if prev_gps_cords:
                self.G.add_edge(prev_gps_cords, gps_cords)
            prev_gps_cords = gps_cords        
            
            #The node in the graph thats added last 
            last_node = list(self.G.nodes())[-1]
            
            #The current direction that the car is moving is identified
            if len(self.G.nodes()) > 30:
                prev_node = list(self.G.nodes())[-15]
                car_md = self.get_car_direction(last_node,prev_node)
            
            """
            Dijkstra path planning is a shortest-path algorithm used for finding the shortest path
            between two nodes in a network graph. It works by iteratively selecting the node with the
            smallest distance from the starting node, updating the distances of its neighbors, 
            and repeating until the destination node is reached. Network Graph creation for Path planning
            (RAS Unit 11 - Path and Motion Planning)
            Referenced - https://stackoverflow.com/questions/59230049/find-the-shortest-distance-between-sets-of-gps-coordinates-python. No code snippet was copied directly
            """
            #If battery is low, then identify the shortest path in the graph to the destination
            if self.time_to_live < low_battery and self.Turn == False: #Not to take shortest path when the turn is already set  
                #print(self.Path)
                #print('low battery')
                if self.Path == None:
                    strsource = '({0}, {1})'.format(round(lat), round(long))
                    if graph.has_node(strsource):
                        self.Path = nx.shortest_path(graph, source=strsource, target=destination, weight='weight')
            else:
                self.Path = None
    
            if self.Path !=None:
                print('shortest path to destination identified')
      
            #detect the road from the roaddetector
            mask_yellow = self.detect_lanes(image)
            
            #calculate the steering angle of the car based on the above identified road, last node and car direction
            steering_angle = self.calculate_steering_angle(mask_yellow, image, last_node,car_md)
            
            #If battery low threshold is reached at a intersection, the direction of turn is set and so the identified 
            #shortest path may not be the correct one. In this scenario, find the shortest path again
            if self.Path != None:
                upsource = '({0}, {1})'.format(round(lat), round(long))
                if graph.has_node(upsource):
                    self.Path = nx.shortest_path(graph, source=upsource, target=destination, weight='weight')
                
            
            #print(self.counter)
            #control the speed of car based on the steering angle. during turns reduce speed.
            if steering_angle == self.RightTurn or steering_angle == self.LeftTurn:
                speed = self.TurnSpeed
            else:
                speed = self.NormalSpeed 
                
            # edge cases, the angle for right turn when car is moving in X axis up the gradient is insufficient to complete the turn
            #therefore adjusting the right turn angle to complete the turn    
            if car_md == 'X' and self.Turn == True:
                if int(prev_node[1]) > int(last_node[1]):
                    if steering_angle == self.RightTurn and self.time_to_live < highbattery - 24:
                        steering_angle = self.RightTurnAdjusted

            #to increase speed when the battery is low and destination is closer
            if self.time_to_live < low_battery:
                strsource = '({0}, {1})'.format(round(lat), round(long))
                if graph.has_node(strsource):
                    dis_destination = nx.shortest_path_length(graph, source=strsource, target=destination)
            if dis_destination < 610 and dis_destination > 50:
                speed = 90

            #set the speed and steering angle of the car
            self.set_speed(speed)    
            self.set_steering_angle(steering_angle)
            print(f'Time to live: {self.time_to_live}')
   
            # Display the image
            cv2.imshow('output', mask_yellow)
            cv2.waitKey(1)
            
            #If in explore mode then save the graph
            if self.ExploreMode:
                self.creategraph() 
                

# Create an instance of the MyRobot class and let it run
robot = MyRobot()
robot.run()
