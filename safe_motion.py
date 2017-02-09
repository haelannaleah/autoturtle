import cv2
import numpy as np

from sensor_msgs.msg import Image

from logger import Logger
from motion import Motion
from sensors import Sensors

class SafeMotion(Motion):
    
    def __init__(self):
        Motion.__init__(self)
        self.sensors = Sensors()
    
    def _avoidance(self, func, *args, **kwargs):
        """ Both be safe and premptive. """
        # if we see a cliff or get picked up, stop
        if self.sensors.cliff or self.sensors.wheeldrop:
            Motion.stop(self, now)
    
        # if we hit something, stop
        elif self.sensors.bump:
            if self.walking:
                Motion.stop_linear(self)
            else:
                Motion.turn(self, self.sensors.bumper > 0)
        
        # otherwise, we keep going
        else:
            func(self, *args, **kwargs)

    def _safety(self, func, *args, **kwargs):
        """ The generic safety wrapper for the Turtlebot. """
        # if we see a cliff or get picked up, stop
        if self.sensors.cliff or self.sensors.wheeldrop:
            Motion.stop(self, now)

        # if we hit something, stop
        elif self.sensors.bump:
            Motion.stop_linear(self)
        
        # otherwise, we keep going
        else:
            func(self, *args, **kwargs)

    def walk(self, speed=1):
        self._safety(Motion.walk, speed)

    def turn(self, speed=1):
        self._safety(Motion.turn, speed)

    def stop(self, now=False):
        self._safety(Motion.turn, now)

    def linear_stop(self, now=False):
        self._safety(Motion.linear_stop, now)

    def rotational_stop(self, now=False):
        self._safety(Motion.rotational_stop, now)

    def shutdown(self):
        self._safety(Motion.shutdown)

if __name__ == "__main__":
    from tester import Tester
    from sensors import Sensors

    class SafeMotionTest(Tester):
        """ Run unit test for the motion class. """
        
        def __init__(self):
            # set up basic sensing
            self.motion = SafeMotion()
            
            Tester.__init__(self, "SafeMotion")

        def main(self):
            self.walk()

        def shutdown(self):
            self.motion.shutdown()

SafeMotionTest()