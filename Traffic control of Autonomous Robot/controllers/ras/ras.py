from rasrobot import RASRobot
from roaddetector import RoadDetector
import numpy as np
import time
import cv2
import yolov5
import torch

# Mission 2: Object detection
"""
The agent is reactive. Therefore once it senses the object, it takes the corresponding action(SENSE-ACT)
(RAS unit 3 Software organisation).  Yolov5s pretrained model to sense the stop sign and based on the
availability of stop sign, stop the car for 1 second.
"""
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

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
        self.counter  = 0
        
    def detect_stop_sign(self, image):
        """
        Detects if a stop sign is present in the given image

        Input: 
            image: a numpy array representing the image
            
        Returns:
            a boolean indicating whether a stop sign is present in the image
        """
        # Use the pretrianed Yolov5 object detection model to detect objects in the image. affects the speed of the car though.
        results = model(image[:, :, ::-1])

        # Check if any of the detected objects in results is a stop sign, and return True, else return False
        class_names = results.names
        boxes = results.xyxy[0].tolist()
        #print(class_names)
        for box in boxes:
            class_id, conf = int(box[5]), box[4]
            if class_names[class_id] == 'stop sign':
                 return True
        return False
        
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
        stop_detected = False
        stop_counter = 0
        while self.tick():
            self.counter += 1
            
            image = self.get_camera_image()
            
            #call to the RoadDetector to idnetify the road and get the necessary masked image that would be used for steering angle calculation
            mask_yellow = self.detect_lanes(image)
            
            #Calculate the steering angle based on the road returned by the RoadDetector 
            steering_angle = self.calculate_steering_angle(mask_yellow, image)
            
            #For every 15 ticks the image is verified for a stop sign and stopped.
            # Set the speed of the robot based on the steering angle, During Turn speed is reduced to prevent car from toppling            
            if self.counter % 15 == 0 and self.detect_stop_sign(image):
                stop_detected = True
                stop_counter = self.counter
                #print(stop_detected,stop_counter)
            
            #As the image is scanned for every 15 ticks, the stop sign may be detected earlier,
            #so to avoid stopping car too early, car is stopped after 10 ticks after detection of stop sign
            #while the stop sign may not be within the camera frame, car will be stopped before the stop sign    
            if stop_detected == True and self.counter == stop_counter + 10:
                stop_detected = False
                stop_counter = 0
                speed = 0
                self.set_speed(speed)
                self.set_steering_angle(0)
                
                #As per the coursework specification for task 2, stopping the car for 1 sec before resuming. 
                time.sleep(1)                
                print('stopping for STOP sign')
            elif steering_angle == self.RightTurn:
                speed = self.TurnSpeed
            else:
                speed = self.NormalSpeed
    
    
            # Set the speed and steering angle of the robot
            self.set_speed(speed)
            self.set_steering_angle(steering_angle)
            
            # Display the lane mask in a window
            cv2.imshow('output', image)
            cv2.waitKey(1)

# Create an instance of the MyRobot class and let it run
robot = MyRobot()
robot.run()
