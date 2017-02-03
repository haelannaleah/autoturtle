""" Unit testing base class.
    
Author:
    Annaleah Ernst
"""

import rospy

from logger import Logger
from motion import Motion

class Tester():
    """
    A module designed as the base class for unit tests using ROS.
    
    Args:
        name (str): The name of the module under test.
        
    Attributes:
        logger (Logger): A custom logger for the tester.
        rate (rospy.Rate): Used to control the refresh rate of robot control loops.
    """
    def __init__(self, name):
        # set module name
        self.__name__ = str(name) + "Tester"
        
        # initialize test node
        rospy.init_node(self.__name__, anonymous = False)
        
        # set up ctrl-C shutdown behavior
        rospy.on_shutdown(self.shutdown)
        
        # set global test refresh rate (100 Hz)
        self.rate = rospy.Rate(100)
    
        # set up test node logger
        self.logger = Logger(self.__name__)
        self.logger.info("hello world")

    def shutdown(self):
        self.logger.info("goodbye world")