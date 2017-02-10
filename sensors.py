""" Low level robot sensors.
    
Author:
    Annaleah Ernst
"""
import rospy

from cv_bridge import CvBridge
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from sensor_msgs.msg import Image

from logger import Logger
from obstacle_detection import ObstacleDetector

class Sensors():
    """ Basic low level sensor suite for the Turtlebot.
        
    Attributes:
        bump (bool): True if robot has bumped into something, False otherwise.
        bumper (int): -1 if left bumper, 0 if middle bumper, 1 if right bumper.
        cliff (bool): True if robot is near an edge, False otherwise.
        cliff_sensor (int): -1 if left sensor, 0 if middle sensor, 1 if right sensor.
        wheeldrop (bool): True if robot is not on the ground, False otherwise.
    """
    _SENSOR_LOCATION = ["Left", "Center", "Right"]
    
    def __init__(self):
        self._logger = Logger("Sensors")

        # subscribe to bump sensor
        self.bump = False
        self.bumper = 0
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self._bumperCallback)

        # subscribe to cliff sensor
        self.cliff = False
        self.cliff_sensor = 0
        rospy.Subscriber('mobile_base/events/cliff', CliffEvent, self._cliffCallback)
        
        # subscribe to depth image
        self.obstacle = False
        self.obstacle_dir = None
        self.obstacleDetector = ObstacleDetector()
        self._bridge = CvBridge()
        rospy.Subscriber('/camera/depth/image', Image, self._depthCallback)

        # subscribe to wheel drop sensor
        self.wheeldrop = False
        rospy.Subscriber('mobile_base/events/wheel_drop', WheelDropEvent, self._wheelDropCallback)

    def _bumperCallback(self, data):
        """ Handle bump events. """
        self.bump = bool(data.state == BumperEvent.PRESSED)
        self.bumper = data.bumper - 1
        self._logKobuki("bumper", data.state, ("RELEASED", "PRESSED"), data.bumper)

    def _cliffCallback(self, data):
        """ Handle cliffs. """
        self.cliff = bool(data.state == CliffEvent.CLIFF)
        self.cliff_sensor = data.sensor - 1
        
        # only log cliff data if it's independent of a wheel drop
        if not self.wheeldrop:
            self._logKobuki("cliff", data.state, ("FLOOR", "CLIFF"), data.sensor)

    def _depthCallback(self, data):
        """ Process incoming depth data. """
        # get the depth image
        self.depth_img = self._bridge.imgmsg_to_cv2(data, 'passthrough')

        # detect obstacles
        if self.obstacleDetector.extractObstacle(self.depth_img) is False:
            self._logger.error("Encountered all NaN slice in depth image.")
    
        elif self.obstacleDetector.obstacle and not self.obstacle:
            self._logger.warn("Encountered obstacle on the " + ["right.", "left."][min_index[1] < w_center])
    
        self.obstacle = self.obstacleDetector.obstacle
        self.obstacle_dir = self.obstacleDetector.obstacle

    def _wheelDropCallback(self, data):
        """ Handle wheel drops. """
        self.wheeldrop = bool(data.state == WheelDropEvent.DROPPED)
        self._logKobuki("WheelDrop", data.state, ("RAISED", "DROPPED"))

    def _logKobuki(self, sensor, state, states, sensor_location = None):
        """ Log useful Kobuki information. """
        if sensor_location is not None:
            sensor = self._SENSOR_LOCATION[sensor_location] + " " + sensor
        
        self._logger.warn(sensor + " event: " + states[state])

if __name__ == "__main__":
    from tester import Tester

    class SensorsTest(Tester):
        """ Behavioral tests for ObstacleDectection. """
        def __init__(self):
            self.sensors = Sensors()
            Tester.__init__(self, "Sensors")

        def main(self):
            self.rate.sleep()

    SensorsTest()
