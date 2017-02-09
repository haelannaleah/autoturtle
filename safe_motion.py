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
        self.avoiding = True
    
        self._safetyModifier = self._avoidance
    
    def _avoidance(self, func, *args, **kwargs):
        """ Both be safe and premptive. """
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
            
        elif self.avoiding:
            if self.walking or self.turning:
                Motion.stop(self)
            else:
                self.avoiding = False
        
        # otherwise, we keep going
        else:
            func(self, *args, **kwargs)

    def _safety(self, func, *args, **kwargs):
        """ The generic safety wrapper for the Turtlebot. """
        # if we see a cliff or get picked up, stop
        if self.sensors.cliff or self.sensors.wheeldrop:
            Motion.stop(self, now=True)

        # if we hit something, stop
        elif self.sensors.bump:
            Motion.stop_linear(self, now=True)
        
        # otherwise, we keep going
        else:
            func(self, *args, **kwargs)

    def walk(self, speed=1):
        self._safetyModifier(Motion.walk, speed)

    def turn(self, speed=1):
        self._safetyModifier(Motion.turn, speed)

    def stop(self, now=False):
        self._safetyModifier(Motion.turn, now)

    def linear_stop(self, now=False):
        self._safetyModifier(Motion.linear_stop, now)

    def rotational_stop(self, now=False):
        self._safetyModifier(Motion.rotational_stop, now)

    def shutdown(self, rate):
        self._safety(Motion.shutdown, rate)

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
            self.motion.walk()

        def shutdown(self):
            self.motion.shutdown(self.rate)

SafeMotionTest()