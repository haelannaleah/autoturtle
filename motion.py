""" Basic motion.
    
Author:
    Annaleah Ernst
"""
import rospy

from geometry_msgs.msg import Twist
from math import radians
from time import time

class Motion():
    """ Handle basic Turtlebot motion. 
    
    Attributes:
        turn_dir (int): 1 if turning left, -1 if turning right, None if no turn.
        turning (bool): True if the robot is turning, False otherwise.
        walking (bool): True if robot is moving linearly, False otherwise.
    """
    
    # Define Turtlebot constants
    _ROT_SPEED = radians(60)
    _LIN_SPEED = 0.2
    _ACCEL_TIME = 0.1
    _ROT_ACCEL = .3
    _ROT_DECCEL = -.35
    _LIN_ACCEL = .025
    _LIN_DECCEL = -.03
    _TURN_LEFT = 1
    _TURN_RIGHT = -1
    
    def __init__(self):

        self.turn_dir = 0
        self.turning = False
        self.walking = False
        self._accel_time = False
        
        # set up publisher/subscriber
        self._move_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self._move_cmd = Twist()
    
    def _accelerate(self, delta):
        """ Smooth out starts and stops. 
            
        Args:
            delta (float): The change in robot speed at each time step. Positive to accelerate,
                negative to accelerate.
        
        Returns:
            Delta if it is time to increment, 0 otherwise.
        """
        # initialize the acceleration time
        if self._accel_time is False:
            self._accel_time = time()
        
        # otherwise, if it's time to increment speed...
        elif time() - self._accel_time > self._ACCEL_TIME:
            self._accel_time = False
            return delta

        return 0
    
    def _linear_stop(self, now):
        """ Gently stop forward motion of robot. """
        if self._move_cmd.linear.x <= 0 or now:
            self._move_cmd.linear.x = 0
            self.walking = False
        else:
            self._move_cmd.linear.x += self._accelerate(self._LIN_DECCEL)

    def _rotational_stop(self, now):
        """ Stop the robot and handle associated housekeeping. """
        if self.turn_dir * self._move_cmd.angular.z <= 0 or now:
            self.turning = False
            self.turn_dir = 0
            self._move_cmd.angular.z = 0
        else:
            self._move_cmd.angular.z += self._accelerate(self.turn_dir * self._ROT_DECCEL)
    
    def _publish(self):
        """ Output commands to the Turtlebot. """
        self._move_publisher.publish(self._move_cmd)

    def stop(self, now=False): 
        """ Stop the robot, immediately if necessary.
        
        Args:
            now (bool): Robot stops immediately if true, else decelerates.
        """
        self._linear_stop(now)
        self._rotational_stop(now)
    
        self._publish()

    def stop_linear(self, now=False):
        """ Stop robot's linear motion. """
        self._linear_stop(now)
        self._publish()

    def stop_rotation(self, now=False):
        """ Stop the robot rotation. """
        self._rotational_stop(now)
        self._publish()

    def turn(self, direction, speed = 1):
        """ Turn the Turtlebot in the desired direction.
            
        Args:
            direction (bool): Turn direction is left if True, right if False
            speed (float, optional): The percentage of the the maximum turn speed
                the robot will turn at.
        """
        # set turn direction
        self.turning = True
        if self.turn_dir is 0:
            self.turn_dir = self._TURN_LEFT if direction else self._TURN_RIGHT

        target_speed = self._ROT_SPEED * min(speed, 1)
        if abs(self._move_cmd.angular.z) < target_speed:
            self._move_cmd.angular.z += self._accelerate(self.turn_dir * self._ROT_ACCEL)
        else:
            self._move_cmd.angular.z = self.turn_dir * target_speed
        
        self._publish()

    def walk(self, speed=1):
        """ Move straight forward. 
        
        Args:
            speed (float, optiona): The percentage of the the maximum linear speed
                the robot will move at.
        """
        self.walking = True
        target_speed = self._LIN_SPEED * min(speed, 1)
        
        if self._move_cmd.linear.x < target_speed:
            self._move_cmd.linear.x += self._accelerate(self._LIN_ACCEL)
        else:
            self._move_cmd.linear.x = target_speed
    
        self._publish()

    def shutdown(self, rate):
        """ Bring the robot to a gentle stop. 
        
        Args:
            rate (rospy.Rate): The refresh rate of the enclosing module.
        """
        while self.walking or self.turning:
            self.stop()
            rate.sleep()

if __name__ == "__main__":
    from tester import Tester
    from sensors import Sensors

    class MotionTest(Tester):
        """ Run unit test for the motion class. """
        
        def __init__(self):
            # set up basic sensing
            self.sensors = Sensors()
            self.motion = Motion()
            
            Tester.__init__(self, "Motion")
        
        def main(self):
            """ The main control loop. """
            self.wander()
        
        def circle(self):
            """ Test simultaneous rotation and forward motion. """
            self.motion.turn(True, 0.5)
            self.motion.walk(0.5)
            
        def wander(self):
            """ Test general safe wandering behavior. """
            # if we see a cliff or get picked up, stop
            if self.sensors.cliff or self.sensors.wheeldrop:
                self.motion.stop(now=True)
        
            # if we hit something, stop and turn
            elif self.sensors.bump:
                if self.motion.walking:
                    self.motion.stop(now=True)
                self.motion.turn(self.sensors.bumper > 0)
            
            # otherwise, just walk
            else:
                if self.motion.turning:
                    self.motion.stop_rotation()
                else:
                    self.motion.walk()

        def shutdown(self):
            """ Shutdown test. """
            self.motion.shutdown(self.rate)
            Tester.shutdown(self)
                
    MotionTest()