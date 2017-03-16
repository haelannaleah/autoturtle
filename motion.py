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
        stopping (bool): True if the robot is in the process of stopping, False otherwise.
        starting (bool): True if the robot is in the process of starting, False otherwise.
    """
    # Define Turtlebot constants
    _ROT_SPEED = radians(60)    # maximum rotational speed
    _LIN_SPEED = 0.2            # maximum linear speed
    _ACCEL_TIME = 0.2           # maximum time delta (s) between acceleration steps
    _ROT_ACCEL = 1.5            # rotational acceleration (m/s^2)
    _ROT_DECCEL = -4            # rotational acceleration (m/s^2)
    _LIN_ACCEL = .2             # linear acceleration (m/s^2)
    _LIN_DECCEL = -.25          # linear deceleration (m/s^2)
    _TURN_LEFT = 1              # positive turn values cause the robot to turn left
    _TURN_RIGHT = -1            # negative turn values cause the robot to turn right
    
    def __init__(self):

        # initialize class attributes
        self.turn_dir = 0
        self.turning = False
        self.walking = False
        self.stopping = False
        self.starting = False
        
        # set up private acceleration trackers
        self._accel_time = 0
        self._prev_accel_time = 0
        
        # set up communication with the robot
        self._move_cmd = Twist()
        self._move_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    
    def _accelerate(self, accel):
        """ Smooth out starts and stops. 
            
        Args:
            accel (float): The change in robot speed in meters per second or radians per second.
                Positive to accelerate, negative to accelerate.
        
        Returns:
            The delta necessary to accelerate at accel.
        """
        # save the previous acceleration time, update the current time, compute the delta
        self._prev_accel_time = self._accel_time
        self._accel_time = time()
        delta_time = self._accel_time - self._prev_accel_time
        
        # we've just begun accelerating since it's been too long since the last step
        if delta_time > self._ACCEL_TIME:
            return 0
        
        # multiply the delta by the desired acceleration to gently and return
        #   this computed delta is used to gently change robot velocity
        return delta_time * float(accel)
    
    def _linearStop(self, now):
        """ Gently stop forward motion of robot. """
        
        # if we've slowed to or past 0, or the user wants to stop now, zero the linear command and reset states
        if self._move_cmd.linear.x <= 0 or now:
            self._move_cmd.linear.x = 0
            self.walking = False
            self.stopping = False
        
        # otherwise, decelerate
        else:
            self.stopping = True
            self._move_cmd.linear.x += self._accelerate(self._LIN_DECCEL)

        self.starting = False

    def _rotationalStop(self, now):
        """ Stop the robot and handle associated housekeeping. """
        
        # if we've slowed to or past 0, or the user wants to stop now, zero the rotational command and reset states
        if self.turn_dir * self._move_cmd.angular.z <= 0 or now:
            self.turning = False
            self.turn_dir = 0
            self._move_cmd.angular.z = 0
        
        # otherwise, decelerate
        else:
            self._move_cmd.angular.z += self._accelerate(self.turn_dir * self._ROT_DECCEL)
    
    def _publish(self):
        """ Output commands to the Turtlebot. """
        
        self._move_publisher.publish(self._move_cmd)
    
    def linearVel(self):
        """ Get the current linear speed of the robot. """
        
        return self._move_cmd.linear.x
    
    def angularVel(self):
        """ Get the current angular velocity of the robot. """
            
        return self._move_cmd.angular.z

    def stop(self, now=False): 
        """ Stop the robot, immediately if necessary.
        
        Args:
            now (bool): Robot stops immediately if true, else decelerates.
        """
        self._linearStop(now)
        self._rotationalStop(now)
    
        self._publish()

    def stopLinear(self, now=False):
        """ Stop robot's linear motion, immediately if necessary. 
        
        Args:
            now (bool): Robot's forward motion stops immediately if true, else decelerates.
        """
        self._linearStop(now)
        self._publish()

    def stopRotation(self, now=False):
        """ Stop the robot rotation, immediately if necessary. 
        
        Args:
            now (bool): Robot's rotational motion stops immediately if true, else decelerates.
        """
        self._rotationalStop(now)
        self._publish()

    def turn(self, direction, speed = 1):
        """ Turn the Turtlebot in the desired direction.
            
        Args:
            direction (bool): Turn direction is left if True, right if False
            speed (float, optional): Percentage of maximum speed, magnitude between 0 and 1.
                Values with magnitude greater than 1 will be ignored.
        """
        # set turning state
        self.turning = True
        
        # set turn direction if it is unset
        if self.turn_dir is 0:
            self.turn_dir = self._TURN_LEFT if direction else self._TURN_RIGHT

        # compute the target speed based on user input (don't let them exceed 1)
        target_speed = self._ROT_SPEED * min(abs(speed), 1)
        
        # if we're less than the target rotational speed, accelerate rotations
        if abs(self._move_cmd.angular.z) < target_speed:
            self._move_cmd.angular.z += self._accelerate(self.turn_dir * self._ROT_ACCEL)
        
        # otherwise, set the rotation speed to the target
        else:
            self._move_cmd.angular.z = self.turn_dir * target_speed
        
        self._publish()

    def walk(self, speed=1):
        """ Move straight forward. 
        
        Args:
            speed (float, optional): Percentage of maximum speed, magnitude between 0 and 1.
                Values with magnitude greater than 1 will be ignored.
        """
        # set walking and stopping states
        self.walking = True
        self.stopping = False
        
        # compute target speed based on user input ()
        target_speed = self._LIN_SPEED * min(abs(speed), 1)
        
        # if we're under our target speed, accelerate
        if self._move_cmd.linear.x < target_speed:
            self._move_cmd.linear.x += self._accelerate(self._LIN_ACCEL)
            self.starting = True
        
        # otherwise, set move command to target speed
        else:
            self.starting = False
            self._move_cmd.linear.x = target_speed
    
        self._publish()

    def shutdown(self, rate):
        """ Bring the robot to a gentle stop. 
        
        Args:
            rate (rospy.Rate): The refresh rate of the enclosing module.
        """
        # if the robot is walking or turning, bring it to a gentle stop
        while self.walking or self.turning:
            self.stop()
            rate.sleep()

if __name__ == "__main__":
    from tester import Tester
    from sensors import Sensors

    class MotionTest(Tester):
        """ Run unit test for the motion class. """
        
        def __init__(self):
            Tester.__init__(self, "Motion")
            
            # set up basic sensing
            self.sensors = Sensors()
            self.motion = Motion()
        
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
                    self.motion.stopRotation()
                else:
                    self.motion.walk()

        def shutdown(self):
            """ Shutdown test. """
            self.motion.shutdown(self.rate)
            Tester.shutdown(self)
                
    MotionTest().run()