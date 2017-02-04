""" Low level robot sensors.
    
Author:
    Annaleah Ernst
"""
import rospy

from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent

from logger import Logger

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
        rospy.Subscriber('mobile_base/events/clif', CliffEvent, self._cliffCallback)

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
        if self.wheeldrop:
            return
        
        self.cliff = bool(data.state == CliffEvent.CLIFF)
        self.cliff_sensor = data.sensor - 1
        self._logKobuki("cliff", data.state, ("FLOOR", "CLIFF"), data.sensor)

    def _wheelDropCallback(self, data):
        """ Handle wheel drops. """
        self.wheeldrop = bool(data.state == WheelDropEvent.DROPPED)
        self._logKobuki("WheelDrop", data.state, ("RAISED", "DROPPED"))

    def _logKobuki(sensor, state, states, sensor_location = None):
        """ Log useful Kobuki information. """
        if sensor_location is not None:
            sensor = self._SENSOR_LOCATION[sensor_location] + " " + sensor
        
        self._logger.warn(sensor + " event: " + states[state])
