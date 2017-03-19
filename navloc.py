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
from time import time

from localization import Localization
from logger import Logger
from navigation import Navigation

class NavLoc(Navigation, Localization):
    """ Navigate and localize on a map.
    
    Args:
        point_ids (set): Unique identifier for each waypoint in the graph.
        locations (dict): Point_ids mapped to tuples representing locations.
        neighbors (dict): Point_ids mapped to lists containing other point_ids representing 
            the current node's neighbors.
        landmark_ids (set): Unique identifier for each landmark in the graph.
        landmark_positions (dict): Map AprilTag landmark ids to their absolute
            position on the floorplan.
        landmark_angles (dict): Map AprilTag landmark ids to their absolute
            position on the floorplan. This specifies the angle of rotation of the landmark in the 
            xy plane; ie, how much has its horizontal vector deviated from the x axis.
        jerky (bool, optional): If true, robot will not decelerate, but stop abruptly.
            Defaults to False.
        walking_speed (float, optional): Percentage of maximum speed, magnitude between 0 and 1.
                Values with magnitude greater than 1 will be ignored.
    
    Attributes:
        tags (geometry_msgs.msg.PoseStamped dict): A dict of all the AprilTags currently in view in 
            their raw form.
        tags_odom (geometry_msgs.msg.PoseStamped dict): Same as above, but in the odometry frame.
        floorplan (FloorPlan): The map of the current space as a floorplan.
        p (geometry_msgs.msg.Point): The position of the robot in the ekf odometry frame according to
            the robot_pose_ekf package.
        q (geometry_msgs.msg.Quaternion): The orientation of the robot in the ekf odometry frame
            according the the robot_pose_ekf package.
        angle (float): The angle (in radians) that the robot is from 0 in the ekf odometry frame. 
            Between -pi and pi
        map_pos (geometry_msgs.msg.Point): The position of the robot in the map frame.
        map_angle (float): The angle (in radians) of the robot in the map frame.
    """
    
    def __init__(self, point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles, jerky = False, walking_speed = 1):
        
        # create map position
        self.map_pos = Point()
        self.map_angle = 0
        
        # create a path variable so that we can navigate via waypoints
        self._path = None
    
        # initialize what we're inheriting from
        Localization.__init__(self, point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles)
        Navigation.__init__(self, jerky = jerky, walking_speed = walking_speed)

        self._logger = Logger("NavLoc")
    
        # give ourselves a second to see if there's a nearby AR tag
        timer = time()
        while time() - timer < 0.5:
            pass
    
    def _ekfCallback(self, data):
        """ Process robot_pose_ekf data. """
        
        # get the ekf data
        Navigation._ekfCallback(self, data)
        
        # compute map data
        self.map_pos = self.transformPoint(self.p, "odom", "map")
        self.map_angle = self.transformAngle(self.angle, "odom", "map")
    
    def _handleObstacle(self, turn_delta):
        """ Handle obstacle and reset path if necessary. """
        
        if Navigation._handleObstacle(self, turn_delta):
            self._path = None
            return True
            
        return False
    
    def goToOrientation(self, angle):
        """ Go to orientation in the map frame. """
        return Navigation.goToOrientation(self, self.transformAngle(angle, "map", "odom"))
    
    def takePathToDest(self, x, y):
        """ Go the target pos via waypoints from the floorplan. 
        
        Args:
            x (float): The destination x coord in the map frame.
            y (float): The destination y coord in the map frame.
        """
        
        # we currently aren't on a mission, or we've been interrupted
        if self._path is None:
            self._path = self.floorplan.getShortestPath(self.map_pos, Point(x,y,0))
        
        # we've arrived a waypoint on our path to destination
        if self.goToPosition(self._path[0].x, self._path[0].y):
            self._logger.info("Arrived at waypoint " + str((self._path[0].x, self._path[0].y)) + " (map position is " +
                str((self.map_pos.x, self.map_pos.y)) + ")")
            self._path.pop(0)
            
        # we've cleared out the traversal path, so we've reached our goal
        if not self._path:
            self._path = None
            self._logger.debug("no path!")
            return True
        
        # we're still on our way to the destination
        return False
    
    def goToPosition(self, x, y):
        """ Go to position x, y, in the map frame"""
        transformed_point = self.transformPoint(Point(x, y, 0), "map", "odom")
        return Navigation.goToPosition(self, transformed_point.x, transformed_point.y)

    def csvLogArrival(self, test_name, x, y, folder = "tests"):
        """ Log the arrival of the robot at a waypoint. """
        
        self._logger.csv(test_name + "_waypoints", ["X_target", "Y_target", "X_map", "Y_map", "X_ekf", "Y_ekf"],
                    [x, y, self.map_pos.x, self.map_pos.y, self.p.x, self.p.y],
                    folder = folder)

    def csvLogMap(self, test_name, folder = "tests"):
        """ Log map position data. """
         
        self._logger.csv(test_name + "_mappose", ["X", "Y", "yaw"], [self.map_pos.x, self.map_pos.y, self.map_angle], folder = folder)

if __name__ == "__main__":
    import MD2
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
            self.test_name = "path"
        
            # set up points on map
            point_ids = MD2.points
            locations = MD2.locations
            neighbors = MD2.neighbors
        
            # set map location of the landmark
            landmarks = MD2.landmarks
            landmark_positions = MD2.landmark_pos
            landmark_orientations = MD2.landmark_orient
        
            self.navloc = NavLoc(point_ids, locations, neighbors,landmarks, landmark_positions, landmark_orientations, jerky = self.jerky, walking_speed = self.walking_speed)
        
            # set the destinations
            self.destination = [self.navloc.floorplan.graph['T'].location, self.navloc.floorplan.graph['R'].location]

        def main(self):
            """ The test currently being run. """
            #self.testCCsquare(1)
            #self.testCsquare(1)
            #self.testLine(1.5)
            self.testPath()
            self.navloc.csvLogEKF(self.test_name)
            self.navloc.csvLogMap(self.test_name)
            self.navloc.csvLogTransform(self.test_name)
            self.navloc.csvLogRawTags(self.test_name)
            self.navloc.csvLogOdomTags(self.test_name)
            #self.navloc.takePathToDest(1.5,0)

        def initFile(self, filename):
            """ Write the first line of our outgoing file (variable names). """
            self.test_name = filename + ("jerky" if self.jerky else "smooth")
        
        def logArrival(self, name, x, y):
            self.logger.info("Arrived at " + str((x, y)) + " (map position is " +
                str((self.navloc.map_pos.x, self.navloc.map_pos.y)) + ")")
            self.navloc.csvLogArrival(self.test_name, x, y)
            
        def testPath(self):
            """ Attempt to navigation between two offices"""
            if not self.reached_corner[0]:
                self.reached_corner[0] = self.navloc.takePathToDest(self.destination[0].x, self.destination[0].y)
                if self.reached_corner[0]:
                    self.logArrival("office 1", self.destination[0].x, self.destination[0].y)
                    
            elif self.navloc.takePathToDest(self.destination[1].x, self.destination[1].y):
                self.reached_corner[0] = False
                self.logArrival("office 2", self.destination[1].x, self.destination[1].y)
        
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