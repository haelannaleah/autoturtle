""" Basic motion.
    
Author:
    Annaleah Ernst
"""
import rospy

from geometry_msgs.msg import Twist
from math import radians
from time import time

class Motion():
    """ Handle basic Turtlebot motion. """
    
    # Define Turtlebot constants
    _ROT_SPEED = radians(60)
    _LIN_SPEED = 0.2
    _ACCEL_TIME = 0.1
    _ACCEL_DELTA = 0.025
    _TURN_LEFT = 1
    _TURN_RIGHT = -1
    
    def __init__(self):

        # initialize instance variables
        self.direction = 1
        self.move_cmd = Twist()
        self.accel_time = False
        self.turn_dir = None
        self.walking = False
        
        # set up publisher/subscriber
        self.move_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    def accelerate(self, delta):
        """ Smooth out starts and stops. """
        
        # initialize the acceleration time
        if self.accel_time is False:
            self.accel_time = time()
        
        # otherwise, if it's time to increment speed...
        elif time() - self.accel_time > self._ACCEL_TIME:
            self.move_cmd.linear.x += delta
            self.accel_time = False

    def linear_stop(self):
        """ Gently stop forward motion of robot.
            
        Returns:
            True if the robot has stopped, False otherwise.
        """
        if self.move_cmd.linear.x > 0:
            self.accelerate(-self._ACCEL_DELTA)
            return False
        else:
            self.move_cmd.linear.x = 0
            self.walking = False
            return True

    def stop(self, now=False): 
        """ Stop the robot, immediately if necessary.
        
        Args:
            now (bool): Robot stops immediately if true, else decelerates.
        """
        if not now:
            self.linear_stop()
        else:
            self.walking = False
            self.move_cmd.linear.x = 0
            
        self.move_cmd.angular.z = 0
        self._publish()

    def turn(self, direction, speed = 1):
        """ Turn the Turtlebot in the desired direction.
            
        Args:
            direction (bool): Turn direction is left if True, right if False
        """
        # if we're still moving forward, stop
        if not self.linear_stop():
            self.move_cmd.angular.z = 0
            self.turn_dir = None

        else:
            # set turn direction
            if self.turn_dir is None:
                self.turn_dir = self._TURN_LEFT if direction else self._TURN_RIGHT

            self.move_cmd.angular.z = self.turn_dir * self._ROT_SPEED * min(speed, 1)
        
        self._publish()

    def walk(self):
        """ Move straight forward. """
        
        self.walking = True
        
        if self.move_cmd.linear.x < self._LIN_SPEED:
            self.accelerate(self._ACCEL_DELTA)
        else:
            self.move_cmd.linear.x = self._LIN_SPEED
            
        self.move_cmd.angular.z = 0
        self._publish()
    
    def _publish(self):
        self.move_publisher.publish(self.move_cmd)