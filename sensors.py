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
        obstacle (bool): True if obstacle in distance threshold, False otherwise.
        obstacle_dir (int): -1 if obstacle on the left, 1 if obstacle on the right, 0 if extracting
            obstacle failed.
        wheeldrop (bool): True if robot is not on the ground, False otherwise.
    """
    # the english location of the sensor
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
        self.wall = False
        self.wall_dir = 0
        rospy.Subscriber('mobile_base/events/cliff', CliffEvent, self._cliffCallback)
        
        # subscribe to depth image for obstacle dectection
        self.depth_img = None
        self.obstacle = False
        self.obstacle_dir = 0
        self._obstacleDetector = ObstacleDetector()
        self._bridge = CvBridge()
        rospy.Subscriber('/camera/depth/image', Image, self._depthCallback)

        # subscribe to wheel drop sensor
        self.wheeldrop = False
        rospy.Subscriber('mobile_base/events/wheel_drop', WheelDropEvent, self._wheelDropCallback)

    def _bumperCallback(self, data):
        """ Handle bump events. """
        
        # set bump state to True if the bumper is pressed
        self.bump = bool(data.state == BumperEvent.PRESSED)
        
        # data comes in as 0 for left, 1 for center, 2 for right, but we want
        #   -1 if left bumper, 0 if middle bumper, 1 if right bumper
        self.bumper = data.bumper - 1
        
        # log the bump event
        self._logKobuki("Bumper", data.state, ("RELEASED", "PRESSED"), data.bumper)

    def _cliffCallback(self, data):
        """ Handle cliffs. """
        
        # set cliff state to True if we encounter a cliff
        self.cliff = bool(data.state == CliffEvent.CLIFF)
        
        # data comes in as 0 for left, 1 for center, 2 for right, but we want
        #   -1 if left sensor, 0 if middle sensor, 1 if right sensor.
        self.cliff_sensor = data.sensor - 1
        
        # only log cliff data if it's independent of a wheel drop so as not to overwhelm the log
        if not self.wheeldrop:
            self._logKobuki("Cliff", data.state, ("FLOOR", "CLIFF"), data.sensor)

    def _depthCallback(self, data):
        """ Process incoming depth data. """
        
        # get the depth image
        self.depth_img = self._bridge.imgmsg_to_cv2(data, 'passthrough')

        # detect obstacles using the ObstacleDetector
        if (self._obstacleDetector.extractObstacle(self.depth_img) is False
            or self._obstacleDetector.extractWall(self.depth_img) is False):
            
            self._logger.error("Encountered all NaN slice in depth image.")
    
        # treat obstacle encounter like an event so as not to overwhelm the log
        elif self._obstacleDetector.obstacle and not self.obstacle:
            self._logKobuki("ObstacleDetector", self._obstacleDetector.obstacle_dir < 0, ["RIGHT", "LEFT"])
            
        # ditto for wall dection
        elif self._obstacleDetector.wall and not self.wall:
            self._logKobuki("WallDetector", self._obstacleDetector.obstacle_dir < 0, ["RIGHT", "LEFT"])
            
        if self.wall and not self.obstacle:
            self._logger.error("HUZZAH")
    
        # set obstacle state and direction to match the obstacle detector
        self.obstacle = self._obstacleDetector.obstacle
        self.obstacle_dir = self._obstacleDetector.obstacle_dir
        
        # ditto for walls
        self.wall = self._obstacleDetector.obstacle
        self.wall_dir = self._obstacleDetector.obstacle_dir

    def _wheelDropCallback(self, data):
        """ Handle wheel drops. """
        
        # set wheeldrop state to True if the wheels have dropped
        self.wheeldrop = bool(data.state == WheelDropEvent.DROPPED)
        
        # log the drop event
        self._logKobuki("WheelDrop", data.state, ("RAISED", "DROPPED"))

    def _logKobuki(self, sensor, state, states, sensor_location = None):
        """ Log useful Kobuki information. 
        
        Args:
            sensor (str): The name of the reporting sensor.
            state (int): The current state of the sensor.
            states (list): The possible states for that sensor.
            sensor_location (int, optional): Which sensor got triggered (0, 1, 2; Left, Center, Right).
        """
        # if we have location data, change the sensor name to reflect the which sensor was triggered
        if sensor_location is not None:
            sensor = self._SENSOR_LOCATION[sensor_location] + " " + sensor
        
        # log the event and state
        self._logger.warn(sensor + " event: " + states[state])

if __name__ == "__main__":
    from tester import Tester

    class SensorsTest(Tester):
        """ Behavioral tests for ObstacleDectection. """
        
        def __init__(self):
            Tester.__init__(self, "Sensors")
            self.sensors = Sensors()

        def main(self):
            """ Run behavioral tests. """
            # since it's all callbacks, we can see how the robot responds to being tiggered
            self.rate.sleep()

    SensorsTest().run()
