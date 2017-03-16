""" Navigation and localization
    
Author:
    Annaleah Ernst
"""
import tf
import rospy
import numpy as np

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from math import sin, cos, pi

from localization import Localization
from logger import Logger
from navigation import Navigation

class NavLoc(Navigation, Localization):
    
    def __init__(self, point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles, jerky = False, walking_speed = 1):
        
        # create transformation object
        self._transform = {"map_pos": Point(0,0,0), "map_angle": 0, "ekf_pos": Point(0,0,0), "ekf_angle": 0}
        self.map_pos = Point()
        self.map_angle = 0
    
        # initialize what we're inheriting from
        Navigation.__init__(self, jerky = jerky, walking_speed = walking_speed)
        Localization.__init__(self, point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles)
        
        # create a timer to slow down the amount that we pay attention to landmarks
        self._timer = float('inf')

        self._logger = Logger("NavLoc")

    def _getDestData(self, destination):
        """ Move from current position to desired waypoint in the odomety frame.
            
        Args:
            destination (geometry_msgs.msg.Point): A destination relative to the map origin, in meters.
        
        Returns:
            True if we are close to the desired location 
            0 if the goal is straight ahead
            The difference between the current angle and the desired angle if we are not on course.
                A negative value indicates that the desired angle is that many radians to the left of 
                the current orientation, positive indicates the desired angle is to the right.
        """
        return Navigation._getDestData(self, self.transformPoint(destination, "map", "odom"))
    
    def _ekfCallback(self, data):
        """ Process robot_pose_ekf data. """
        
        # get the ekf data
        Navigation._ekfCallback(self, data)
        
        # compute map data
        self.map_pos = self.transformPoint(self.p, "odom", "map")
        self.map_angle = self.transformAngle(self.angle, "odom", "map")

    def csvLogArrival(self, test_name, x, y, folder = "tests"):
        """ Log the arrival of the robot at a waypoint. """
        
        self._logger.csv(test_name + "_waypoints", ["X_target", "Y_target", "X_map", "Y_map", "X_ekf", "Y_ekf"],
                    [x, y, self.map_pos.x, self.map_pos.y, self.p.x, self.p.y],
                    folder = folder)

    def csvLogMap(self, test_name, folder = "tests"):
        """ Log map position data. """
         
        self._logger.csv(test_name + "_mappose", ["X", "Y", "yaw"], [self.map_pos.x, self.map_pos.y, self.map_angle], folder = folder)

if __name__ == "__main__":
    from tester import Tester
    from math import pi
    
    class NavLocTest(Tester):
        """ Run local navigation tests. """
        def __init__(self):
            Tester.__init__(self, "NavLoc")
            
            # flag for a jerky stop
            self.jerky = False
            
            # I'm a bit concerned about robot safety if we don't slow things down,
            # but I'm also worried it won't be an accurate test if we change the speed
            self.walking_speed = 1 # if not self.jerky else .5
            
            # linear test
            self.reached_goal = False
            
            # square test
            self.reached_corner = [False, False, False, False]
            self.cc_square = [(0,0), (1,0), (1,1), (0,1)]
            self.c_square = [(0,0), (1,0), (1,-1), (0, -1)]
            self.corner_counter = 0
        
            # set up the logger output file
            self.test_name = "angled_line"
        
            # set map location of the landmark
            landmarks = {0}
            landmark_positions = {0:(2.25,0)}
            landmark_orientations = {0:-pi/2}
        
            self.navloc = NavLoc({},{},{},landmarks, landmark_positions, landmark_orientations, jerky = self.jerky, walking_speed = self.walking_speed)

        def main(self):
            """ The test currently being run. """
            #self.testCCsquare(1)
            self.testCsquare(1)
            #self.testLine(1.5)
            self.navloc.csvLogEKF(self.test_name)
            self.navloc.csvLogMap(self.test_name)
            self.navloc.csvLogTransform(self.test_name)
            self.navloc.csvLogRawTags(self.test_name)
            self.navloc.csvLogOdomTags(self.test_name)

        def initFile(self, filename):
            """ Write the first line of our outgoing file (variable names). """
            self.test_name = filename + ("jerky" if self.jerky else "smooth")
            self.logger.csv(self.test_name, ["map_x", "map_y", "reported_x", "reported_y"], folder = "tests")
        
        def logArrival(self, name, x, y):
            self.logger.info("Arrived at " + str((x, y)) + " (map position is " +
                str((self.navloc.map_pos.x, self.navloc.map_pos.y)) + ")")
            self.navloc.csvLogArrival(self.test_name, x, y)
        
        def testLine(self, length):
            """ Test behavior with a simple line. 
            
            Args:
                length (float): Length of the desired line (in meters).
            """
            if self.test_name is None:
                self.initFile("line")
            
            if not self.reached_corner[0]:
                self.reached_corner[0] = self.navloc.goToPosition(0, 0)
                if self.reached_corner[0]:
                    self.logArrival("home", 0, 0)
        
            elif self.navloc.goToPosition(length, 0):
                self.reached_corner[0] = False
                self.logArrival("endpoint", length, 0)
    
        def testCCsquare(self, length):
            """ Test a counter clockwise square. 
            
            Args:
                length (float): Length of the desired line (in meters).
            """
            if self.test_name is None:
                self.initFile("counterclockwise")
            
            self.testSquare(length, self.cc_square)
        
        def testCsquare(self, length):
            """ Test a clockwise square. 
            
            Args:
                length (float): Length of the desired line (in meters).
            """
            if self.test_name is None:
                self.initFile("clockwise")
            
            self.testSquare(length, self.c_square)
    
        def testSquare(self, length, corners):
            """ Test behavior with a simple square. 
            
            Args:
                length (float): Length of the sides of the square (in meters).
            """
            # test a simple square
            if not self.reached_corner[self.corner_counter]:
                self.reached_corner[self.corner_counter] = self.navloc.goToPosition(corners[self.corner_counter][0]*length, corners[self.corner_counter][1]*length)
            
            else:
                self.logArrival("corner " + str(self.corner_counter), corners[self.corner_counter][0]*length, corners[self.corner_counter][1]*length)
                if self.corner_counter == len(self.reached_corner) - 1:
                    self.reached_corner = [False] * len(self.reached_corner)
                self.corner_counter = (self.corner_counter + 1) % len(self.reached_corner)
    
        def shutdown(self):
            """ Kill all behavioral test processes. """
            self.navloc.shutdown(self.rate)
            Tester.shutdown(self)
        
    NavLocTest().run()