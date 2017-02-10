""" Detect obstacles before we collide with them.
    
Author:
    Annaleah Ernst
"""
import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge

from logger import Logger

class ObstacleDetector():
    """ Detect obstacles.
    
    Attributes:
        obstacle (bool): True if there is an obstacle that needs attention, False otherwise.
        obstacle_dir (int): -1 if the obstacle is on the left, 1 if the obstacle is on the right.
    """
    _OBSTACLE_DIST_THRESH =  0.5
    _OBSTACLE_SAMPLE_WIDTH = 0.3

    def __init__(self):
        self._logger = Logger("ObstacleDetector")
        self.bridge = CvBridge()
        
        self.obstacle = False
        self.obstacle_dir = 0

    def _getBlurredDepthSlice(self, depth_img, min_height, max_height, min_width, max_width):
        """ Get a prepped slice of the image. """
        return cv2.medianBlur(depth_img[min_height:max_height, min_width:max_width], 5)
    
    def _getIndex(self, img, operation):
        """ Get the index of the shortest distance in the depth image slice. """
        try:
            return np.unravel_index(operation(img[np.nonzero(img)]), img.shape)
        except ValueError:
            return None
    
    def _extractObstacle(self, depth_img):
        """ If there is an obstacle nearby, find it. """
        # get slice to check distance on
        img_height, img_width = depth_img.shape
        s_width = int(img_width * self._OBSTACLE_SAMPLE_WIDTH)
        w_center = img_width // 2
    
        # apply blur to smooth out irregularities
        sample = self._getBlurredDepthSlice(0, img_height, w_center - s_width, w_center + s_width)

        # check distance to closest object
        min_index = self._getIndex(sample, np.nanargmin)
        if min_index is None:
            # fail as safely as possible
            self.obstacle = True
            return False
        
        # if the closest thing in our slice is too close, likely an obstacle
        if sample[min_index] < self._OBSTACLE_DIST_THRESH:
            if not self.obstacle:
                self.obstacle_dir = -1 if min_index[1] < w_center else 1
                self.obstacle = True
        else:
            self.obstacle = False