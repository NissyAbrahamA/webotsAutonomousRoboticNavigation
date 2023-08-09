from rasrobot import RASRobot
from roaddetector import RoadDetector
import numpy as np
import time
import cv2

"""Mission 1 - Reactive vision based Navigation. 
Since its a Reactive vision based agent, it has been coded to identify the road(presence of yellow line in the
road as the coursework specification mentions that the robot can take any lane and doesnt have to maintain it)
and drive on the road avoiding leaving the tarmac at any time. Its purely reactive, the steering angle is calculated
based on the input image with the yellow line. During intersection(identifying intersection using image as well)
take right turn always. - Reactive agen (RAS unit 3- Software organisation)
"""
class MyRobot(RASRobot, RoadDetector):
    def __init__(self):
        super(MyRobot, self).__init__()

        # Initialise and resize a new window 
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("output", 128*4, 64*4)
        
        #variables initialisation
        self.Turn = False
        self.ContourFound = 0
        self.DimCamera = False
        self.RightTurn = 0.2 #steering angle for right turn, which when applied for individual ticks help finish the right turn
        self.TurnSpeed = 10
        self.NormalSpeed = 40
        
    def calculate_steering_angle(self, mask_yellow, image):
        """
        Calculates the steering angle based on the yellow mask.
        
        The moments of the yellow mask (input masked image) is used to find the center of yellow line and calculate steering angle
        based on it. self.Turn is set to True when there is no contours(moments) identified, meaning its 
        an intersection. When at intersection, according to the coursework for Task 1, The turn should be 
        right. The self.ContourFound and self.DimCamera are used to ignore the pedestrain 
        crossing which will collide with the yellow line in the middle of the line and prevent the car from
        leaving the tarmac, when applicable.
                
        Input:
            mask_yellow (numpy array): Binary mask of the yellow color in the image.
            image (numpy array): Original image to draw the steering angle on.

        Returns:
            float: Steering angle to control the robot's movement.
        """
        height, width, _ = image.shape
        
        #Calculate moments of the yellow mask to determine the center of mass.
        moments = cv2.moments(mask_yellow)
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.circle(image, (cx, cy), 10, (0, 255, 255), -1)
            steering_angle = (cx - width/2) / (width/2)
            self.ContourFound = self.ContourFound + 1
            
            if self.Turn == True and self.ContourFound < 100:
                self.DimCamera = True
            else:
                self.DimCamera = False
                self.ContourFound = 0
                self.Turn = False
        else:
            steering_angle = self.RightTurn
            self.Turn = True
            
        return steering_angle

        
    def run(self):
        """
        This function implements the main loop of the robot.
        """
        while self.tick():
            image = self.get_camera_image()
            
            #call to the RoadDetector to idnetify the road and get the necessary masked image that would be used for steering angle calculation
            mask_yellow = self.detect_lanes(image)
            
            #Calculate the steering angle based on the road returned by the RoadDetector 
            steering_angle = self.calculate_steering_angle(mask_yellow, image)
            
            # Set the speed of the robot based on the steering angle, During Turn speed is reduced to prevent car from toppling            
            if steering_angle == self.RightTurn:
                speed = self.TurnSpeed
            else:
                speed = self.NormalSpeed
            
            # Set the speed and steering angle of the robot
            self.set_speed(speed)
            self.set_steering_angle(steering_angle)
            
            # Display the lane mask in a window
            cv2.imshow('output', mask_yellow)
            cv2.waitKey(1)

# Create an instance of the MyRobot class and let it run
robot = MyRobot()
robot.run()
