""" Safe motion.
    
Author:
    Annaleah Ernst
"""
from logger import Logger
from motion import Motion
from sensors import Sensors

class SafeMotion(Motion):
    """ Handle basic Turtlebot motion while being aware of safety issues.
    
    Args:
        safety_level (int): Indicates the level of interference safety precautions will
            wreak on normal motion. At 0, the robot stops is picked up, bumped, or encounters 
            a cliff.
        
    Attributes:
        avoiding (bool): True if the robot is in avoidance mode, False otherwise.
        turn_dir (int): 1 if turning left, -1 if turning right, None if no turn.
        turning (bool): True if the robot is turning, False otherwise.
        walking (bool): True if robot is moving linearly, False otherwise.
    """
    def __init__(self, safety_level=0):
        Motion.__init__(self)
        self._logger = Logger("SafeMotion")
        
        # set up sensor module
        self.sensors = Sensors()
        
        # set avoidance state
        self.avoiding = False
    
        # set motion modification based on safety level
        self._motionModifier = self._avoid if safety_level > 0 else self._safetyStop
    
    def _avoid(self, func, *args, **kwargs):
        """ Both be safe and premptive. 
        
        Args:   
            func (function): A Turtlebot behavioral function.
            *args: The arguments to func.
            **kwargs: Keyword arguments to func.
        """
        # if we see a cliff or get picked up, stop
        if self.sensors.cliff or self.sensors.wheeldrop:
            Motion.stop(self, now=True)
    
        # if we hit something, stop
        elif self.sensors.bump:
            if self.walking:
                Motion.stop_linear(self, now=True)
            else:
                self.avoiding = True
                Motion.turn(self, self.sensors.bumper > 0)

        # if we see something coming, avoid
        elif self.sensors.obstacle:
            if self.walking:
                Motion.stop_linear(self)
            else:
                self.avoiding = True
                Motion.turn(self, self.sensors.obstacle_dir > 0)
        
        # this means that we're avoiding nothing!
        elif self.avoiding:
            if self.walking or self.turning:
                Motion.stop(self)
            else:
                self.avoiding = False
        
        # otherwise, we keep going
        else:
            func(self, *args, **kwargs)

    def _safetyStop(self, func, *args, **kwargs):
        """ The generic safety wrapper for the Turtlebot. 
        
        Args:   
            func (function): A Turtlebot behavioral function.
            *args: The arguments to func.
            **kwargs: Keyword arguments to func.
        """
        # if we see a cliff or get picked up, stop now
        if self.sensors.cliff or self.sensors.wheeldrop:
            Motion.stop(self, now=True)

        # if we hit something, stop now
        elif self.sensors.bump:
            Motion.stop_linear(self, now=True)
            
        # if we see something coming, stop gently
        elif self.sensors.obstacle:
            Motion.stop_linear(self)
        
        # otherwise, stay the course
        else:
            func(self, *args, **kwargs)

    def stop_linear(self, now=False):
        """ Stop robot's linear motion, immediately if necessary. 
        
        Args:
            now (bool): Robot's forward motion stops immediately if true, else decelerates.
        """
        self._safetyStop(Motion.stop_linear, now)

    def stop_rotation(self, now=False):
        """ Stop the robot rotation, immediately if necessary. 
        
        Args:
            now (bool): Robot's rotational motion stops immediately if true, else decelerates.
        """
        self._safetyStop(Motion.stop_rotation, now)
    
    def stop(self, now=False):
        """ Stop the robot, immediately if necessary.
        
        Args:
            now (bool): Robot stops immediately if true, else decelerates.
        """
        self._safetyStop(Motion.stop, now)
    
    def turn(self, direction, speed=1):
        """ Turn the Turtlebot in the desired direction.
            
        Args:
            direction (bool): Turn direction is left if True, right if False
            speed (float, optional): Percentage of maximum speed, magnitude between 0 and 1.
                Values with magnitude greater than 1 will be ignored.
        """
        self._motionModifier(Motion.turn, direction, speed)

    def walk(self, speed=1):
        """ Move straight forward. 
        
        Args:
            speed (float, optional): Percentage of maximum speed, magnitude between 0 and 1.
                Values with magnitude greater than 1 will be ignored.
        """
        self._motionModifier(Motion.walk, speed)

    def shutdown(self, rate):
        """ Bring the robot to a gentle stop. 
        
        Args:
            rate (rospy.Rate): The refresh rate of the enclosing module.
        """
        self._safetyStop(Motion.shutdown, rate)

if __name__ == "__main__":
    from tester import Tester

    class SafeMotionTest(Tester):
        """ Run unit test for the motion class. """
        
        def __init__(self):
            Tester.__init__(self, "SafeMotion")
            
            # set up basic sensing
            self.motion = SafeMotion(safety_level=1)

        def main(self):
            # running walk here should behave like wander in the motion module test
            self.motion.walk()

        def shutdown(self):
            self.motion.shutdown(self.rate)

    SafeMotionTest().run()