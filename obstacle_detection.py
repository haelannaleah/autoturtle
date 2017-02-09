""" Detect obstacles before we collide with them.
    
Author:
    Annaleah Ernst
"""
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image

from logger import Logger

class ObstacleDetection():
    """ Detect obstacles.
    
    Attributes:
        obstacle (bool): True if there is an obstacle that needs attention, False otherwise.
        obstacle_dir (int): -1 if the obstacle is on the left, 1 if the obstacle is on the right.
    """
    _OBSTACLE_DIST_THRESH =  0.6
    _OBSTACLE_SAMPLE_WIDTH = 0.3

    def __init__(self):
        self._logger = Logger("ObstacleDetection")
        self.bridge = CvBridge()
        
        self.obstacle = False
        self.obstacle_dir = 0
        
        self.depth_img = None
        rospy.Subscriber('/camera/depth/image', Image, self._depthCallback)

    def _getBlurredDepthSlice(self, min_height, max_height, min_width, max_width):
        """ Get a prepped slice of the image. """
        return cv2.medianBlur(self.depth_img[min_height:max_height, min_width:max_width], 5)
    
    def _getMinIndex(self, img):
        """ Get the index of the shortest distance in the depth image slice. """
        try:
            return np.unravel_index(np.nanargmin(img[np.nonzero(img)]), sample.shape)
        except ValueError:
            # fail as safely as possible
            self._logger.error("Encountered all NaN slice in depth image.")
            return None
    
    def _extractObstacle(self):
        """ If there is an obstacle nearby, find it. """
        # get slice to check distance on
        img_height, img_width = self.depth_img.shape
        s_width = int(img_width * self._SAMPLE_WIDTH)
        w_center = img_width // 2
    
        # apply blur to smooth out irregularities
        sample = self._getBlurredDepthSlice(0, img_height, w_center - s_width, w_center + s_width)

        # check distance to closest object
        min_index = self._getMinIndex(self, sample)
        if min_index is None:
            obstacle = True
            return
        
        # if the closest thing in our slice is too close, likely an obstacle
        if sample[min_index] < self._DIST_THRESH:
            if not self.obstacle:
                self.obstacle_dir = -1 if min_index[1] < w_center else 1
                self.obstacle = True
                self._logger.warn("Encountered obstacle on the " + ["right.", "left."][min_index[1] < w_center])
        else:
            self.obstacle = False

    def _depthCallback(self, data):
        """ Process incoming depth data. """
        self._logger.debug("depth callback")
        
        # get the depth image
        self.depth_img = self.bridge.imgmsg_to_cv2(data, 'passthrough')

        # detect obstacles
        self._extractObstacle()

if __name__ == "__main__":
    from tester import Tester

    class ObstacleDetectionTest(Tester):
        """ Behavioral tests for ObstacleDectection. """
        def __init__(self):
            Tester.__init__(self, "ObstacleDetection")

        def main(self):
            self.logger.debug("logging")
            self.rate.sleep()

    ObstacleDetectionTest()