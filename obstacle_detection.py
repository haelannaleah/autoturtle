""" Detect obstacles before we collide with them.
    
Author:
    Annaleah Ernst
"""
import cv2
import numpy as np
import rospy

from logger import Logger

class ObstacleDetector():
    """ Detect obstacles.
    
    Attributes:
        obstacle (bool): True if there is an obstacle that needs attention, False otherwise.
        obstacle_dir (int): -1 if the obstacle is on the left, 1 if the obstacle is on the right.
    """
    # obstacle detection constants
    _OBSTACLE_DIST_THRESH =  0.55   # distance threshold for obstacles to the Turtlebot
    _OBSTACLE_SAMPLE_WIDTH = 0.3    # slice of the robot's vision to check for obstacles
    _WALL_SAMPLE_WIDTH = 0.5

    def __init__(self):
        self._logger = Logger("ObstacleDetector")
        
        # initialize obstacle states and directions
        self.obstacle = False
        self.obstacle_dir = 0
        self.wall = False
        self.wall_dir = 0
    
        # intialize wall detection

    def _getBlurredDepthSlice(self, depth_img, min_height, max_height, min_width, max_width):
        """ Get a blurred slice of the image. 
        
        Args:
            depth_img (numpy matrix): The cv2 depth image that we want to get a slice of.
            min_height (int): The minimum y-pixel of the slice.
            max_height (int): The maximum y-pixel of the slice.
            min_width (int): The minimum x-pixel of the slice.
            max_width (int): The minimum y-pixel of the slice.
            
        Returns:
            A numpy matrix representing blurred depth image sliced to specifications.
        """
        return cv2.medianBlur(depth_img[min_height:max_height, min_width:max_width], 5)
    
    def _getIndex(self, img, operation):
        """ Get the index of the part of the imag that most satisfies the numpy operation.
        
        Args:
            img (numpy matrix): The cv2 depth image that we want to detect obstacles on.
            operation (numpy arg function): A function that performs some operation and returns
                a numpy index.
        
        Returns:
            The np index of the depth image that satisfied the input operation.
        """
        try:
            return np.unravel_index(operation(img[np.nonzero(img)]), img.shape)
        except ValueError:
            return None

    def extractObstacle(self, depth_img):
        """ If there is an obstacle nearby, find it. 
        
        Args:
            depth_img (numpy matrix): The cv2 depth image that we want to detect obstacles on.
            
        Returns:
            True on success, False on failure.
        """
        self.obstacle, self.obstacle_dir = self._extractObstruction(depth_img, self._OBSTACLE_SAMPLE_WIDTH, self.obstacle)
        
        if self.obstacle and self.obstacle_dir == 0:
            return False
        
        return True
        
    def extractWall(self, depth_img):
        """ If there is a wall, or very near obstacle, find it.
        
        Args:
            depth_img (numpy matrix): The cv2 depth image that we want to detect obstacles on.
            
        Returns:
            True on success, False on failure.
        """
        if self.obstacle:
            self.wall, self.wall_dir = True, self.obstacle_dir
        else:
            self.wall, self.wall_dir = self.extractObstacle(depth_img, self._WALL_SAMPLE_WIDTH, self.wall)
        
        if self.wall and self.wall_dir == 0:
            return False
        
        return True
    
    def _extractObstruction(self, depth_img, obstruction_sample_width, cur_state):
        """ If there is an obstacle nearby, find it. 
        
        Args:
            depth_img (numpy matrix): The cv2 depth image that we want to detect obstacles on.
            obstruction_sample_width (numpy matrix): The width of the sample space in the image
            
        Returns:
            True on success, False on failure.
        """
        # compute the bounds of our image slice
        img_height, img_width = depth_img.shape
        s_width = int(img_width * obstruction_sample_width)
        w_center = img_width // 2
    
        # get a slice and apply blur to smooth out irregularities
        sample = self._getBlurredDepthSlice(depth_img, 0, img_height, w_center - s_width, w_center + s_width)

        # check distance to closest object
        min_index = self._getIndex(sample, np.nanargmin)
        
        # if the operation failed (likely due to an all NaN slice), set obstacle state and return failure
        if min_index is None:
            return True, 0
        
        # if the closest thing in our slice is too close, trigger obstacle detection
        if sample[min_index] < self._OBSTACLE_DIST_THRESH:
        
            # if we aren't already in the obstacle state, set state to true and set obstacle direction
            #   if we don't do this, then the obstacle direction is allowed to fluctuate and the robot
            #   behaves irratically; here we are fixing the direction to be the inital trigger for the
            #   current obstacle state
            if not cur_state:
                return (True, -1 if min_index[1] < w_center else 1)
        
        # otherwise, return no obstruction
        return False, 0
