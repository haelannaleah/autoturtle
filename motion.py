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
        turn_dur (int): 1 if turning left, -1 if turning right, None if no turn.
        walking (bool): True if robot is moving linearly, False otherwise.
    """
    
    # Define Turtlebot constants
    _ROT_SPEED = radians(60)
    _LIN_SPEED = 0.2
    _ACCEL_TIME = 0.1
    _ACCEL_DELTA = .003
    _DECEL_DELTA = -.004
    _TURN_LEFT = 1
    _TURN_RIGHT = -1
    
    def __init__(self):

        self.turn_dir = None
        self.walking = False
        self._accel_time = False
        
        # set up publisher/subscriber
        self._move_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self._move_cmd = Twist()
    
    def _linear_stop(self):
        """ Gently stop forward motion of robot.
            
        Returns:
            True if the robot has stopped, False otherwise.
        """
        if self._move_cmd.linear.x > 0:
            self.accelerate(self._DECEL_DELTA)
            return False
        else:
            self._move_cmd.linear.x = 0
            self.walking = False
            return True
    
    def _publish(self):
        self._move_publisher.publish(self._move_cmd)

    def accelerate(self, delta):
        """ Smooth out starts and stops. 
            
        Args:
            delta (float): The change in robot speed at each time step. Positive to accelerate,
                negative to accelerate.
        """
        # initialize the acceleration time
        if self._accel_time is False:
            self._accel_time = time()
        
        # otherwise, if it's time to increment speed...
        elif time() - self._accel_time > self._ACCEL_TIME:
            self._move_cmd.linear.x += delta
            self.accel_time = False

    def stop(self, now=False): 
        """ Stop the robot, immediately if necessary.
        
        Args:
            now (bool): Robot stops immediately if true, else decelerates.
        """
        if not now:
            self._linear_stop()
        else:
            self.walking = False
            self._move_cmd.linear.x = 0
            
        self._move_cmd.angular.z = 0
        self._publish()

    def turn(self, direction, speed = 1):
        """ Turn the Turtlebot in the desired direction.
            
        Args:
            direction (bool): Turn direction is left if True, right if False
        """
        # if we're still moving forward, stop
        if not self._linear_stop():
            self._move_cmd.angular.z = 0
            self.turn_dir = None

        else:
            # set turn direction
            if self.turn_dir is None:
                self.turn_dir = self._TURN_LEFT if direction else self._TURN_RIGHT

            self._move_cmd.angular.z = self.turn_dir * self._ROT_SPEED * min(speed, 1)
        
        self._publish()

    def walk(self, speed=1):
        """ Move straight forward. """
        self.walking = True
        target_speed = self._LIN_SPEED * min(speed, 1)
        
        if self._move_cmd.linear.x < target_speed:
            self.accelerate(self._ACCEL_DELTA)
        else:
            self._move_cmd.linear.x = target_speed
            
        self._move_cmd.angular.z = 0
        self._publish()

    def shutdown(self, rate):
        """ Bring the robot to a gentle stop. """
        while self.walking:
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
            
            # if we see a cliff or get picked up, stop
            if self.sensors.cliff or self.sensors.wheeldrop:
                self.motion.stop(now=True)
        
            # if we hit something, stop and turn
            elif self.sensors.bump:
                self.motion.turn(self.sensors.bumper > 0)
            
            # otherwise, just walk
            else:
                self.motion.walk()

        def shutdown(self):
            """ Shutdown test. """
            self.motion.shutdown(self.rate)
            Tester.shutdown(self)
                
    MotionTest()