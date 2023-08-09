import numpy as np
import cv2

class RoadDetector:
    """
    A class for detecting road lanes in an image using color segmentation 
    and region of interest selection.
    Utilised methods used in https://www.kaggle.com/code/soumya044/lane-line-detection. No code snippet was copied directly.
    """
    def detect_lanes(self, image):
        """
        Detects the yellow line in an image using color segmentation and 
        region of interest selection.
        
        The input image is converted to HSV color space, approriate yellow color segmentation is
        applied on the image and based on the region on interest defined, only the desired view is
        returned
        
        Input:
        - image: RGB image at every instance of the tick execution
        Returns:
        - a binary NumPy array representing the mask, where white pixels 
        indicate the presence of road lane.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_yellow, upper_yellow = self.get_yellow_thresholds()
        
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        mask_yellow = self.define_roi_mask(mask_yellow, image)
               
        return mask_yellow
    
    def get_yellow_thresholds(self):
        """
        This function is to apply appropriate range of color segmentation to ease the car movement.
        Useful for Task 3 and 4 debugging.
        
        Returns:
        - a tuple of two NumPy arrays representing the lower and upper thresholds 
        for the yellow color in the HSV color space based on the input conditions
        """
        if self.Turn == True and self.DimCamera == True:
            lower_yellow = np.array([10, 100, 100])
            upper_yellow = np.array([30, 125, 255])
        else:
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
        return lower_yellow, upper_yellow
    
    def define_roi_mask(self, mask_yellow, image):
        """
        Applies a region of interest mask to a binary mask to extract only the desired lane pixels.
        Desired image is just the road in the immediate vicinity of the car and ignore the road ahead 
        
        Input:
        - mask_yellow: a binary NumPy array representing the yellow lane mask
        - image: a NumPy array representing an RGB image
        
        Returns:
        - a binary NumPy array representing the masked lane mask, where white pixels indicate 
        the presence of a lane in the region of interest
        """
        height, width, _ = image.shape
        #print(width)
        roi_width = width * 0.6 # 60% of image width
        roi_height = height * 0.2 # 20% of image height
        roi_image = np.array([[(width/2 - roi_width/2, height),
                              (width/2 + roi_width/2, height),
                              (width/2, height - roi_height)]], dtype=np.int32)
        mask = np.zeros_like(mask_yellow)
        cv2.fillPoly(mask, roi_image, 255)
        mask_yellow = cv2.bitwise_and(mask_yellow, mask)
        return mask_yellow
    
    
    