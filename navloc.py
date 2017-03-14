""" Navigation and localization
    
Author:
    Annaleah Ernst
"""
import tf
import rospy

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from math import sin, cos, pi
from time import time

from localization import Localization
from logger import Logger
from navigation import Navigation

class NavLoc(Navigation, Localization):
    
    def __init__(self, point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles, jerky = False, walking_speed = 1):
        
        # create transformation object
        self._transform = {"map_pos": Point(0,0,0), "map_angle": 0, "ekf_pos": Point(0,0,0), "ekf_angle": 0, "angle_delta": 0}
        self._raw_pose = Pose()
        self._raw_angle = 0
    
        # initialize what we're inheriting from
        Navigation.__init__(self, jerky = jerky, walking_speed = walking_speed)
        Localization.__init__(self, point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles)
        
        # create a timer to slow down the amount that we pay attention to landmarks
        self._timer = float('inf')

        self._logger = Logger("NavLoc")
    
    def _estimatePose(self):
        """ Override pose estimation to include pose calculation. """
    
        Localization._estimatePose(self)
        
        # if there is currently no estimated pose, nothing more to do here
        if self.estimated_pose is None:
            return
        
        # save current odometry position
        ekf_pose = deepcopy(self._raw_pose)
        ekf_angle = self._raw_angle
        self._transform["ekf_pos"] = ekf_pose.position
        self._transform["ekf_angle"] = ekf_angle

        # save the estimated map position
        self._transform["map_pos"] = self.estimated_pose.position
        self._transform["map_angle"] = self.estimated_angle
        
        # compute the difference between them
        self._transform["angle_delta"] = self._transform["ekf_angle"] - self._transform["map_angle"]
        
    def _ekfCallback(self, data):
        """ Process robot_pose_ekf data. """
        
        # extract raw data
        self._raw_pose = data.pose.pose
        raw_q = self._raw_pose.orientation
        self._raw_angle = tf.transformations.euler_from_quaternion([raw_q.x, raw_q.y, raw_q.z, raw_q.w])[-1]

        # measure the amount that the robot has moved since we last saw a landmark
        ekf_dx = self._raw_pose.position.x - self._transform["ekf_pos"].x
        ekf_dy = self._raw_pose.position.y - self._transform["ekf_pos"].y
        ekf_delta_angle = self._raw_angle - self._transform["ekf_angle"]

        # transform from odom to the map frame
        self.p = Point()
        self.p.x = self._transform["map_pos"].x + ekf_dx * cos(self._transform["angle_delta"]) - ekf_dy * sin(self._transform["angle_delta"])
        self.p.y = self._transform["map_pos"].y + ekf_dx * sin(self._transform["angle_delta"]) + ekf_dy * cos(self._transform["angle_delta"])
        
        self._logger.debug(self._transform)
        
        # since a quaternion respresents 3d space, and turtlebot motion is in 2d, we can just
        #   extract the only non zero euler angle as the angle of rotation in the floor plane
        self.angle = self._transform["map_angle"] + ekf_delta_angle
        self._logger.debug(self.angle, var_name = "angle")
        
        # wrap angle, if necessary
        if self.angle > pi:
            self.angle -= self._TWO_PI
        elif self.angle < -pi:
            self.angle += self._TWO_PI

        # compute the quaternion
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(0, 0, self.angle)
        self.q = Quaternion(qx, qy, qz, qw)

#    def _getDestData(self, destination):
#        # convert destination out of the map frame into the odom frame
#        destination.x = self._transform["position"].x + destination.x * cos(self._transform["angle"]) - destination.y * sin(self._transform["angle"])
#        destination.y = self._transform["position"].y + destination.x * sin(self._transform["angle"]) - destination.y * cos(self._transform["angle"])
#        return Navigation._getDestData(self, destination)

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
            self.c_square = [(0,0), (0,1), (1,1), (1,0)]
            self.corner_counter = 0
        
            # set up the logger output file
            self.filename = None
        
            landmarks = {0}
            landmark_positions = {0:(1.75,0)}
            landmark_orientations = {0:-pi/2}
        
            self.navloc = NavLoc({},{},{},landmarks, landmark_positions, landmark_orientations, jerky = self.jerky, walking_speed = self.walking_speed)

        def main(self):
            """ The test currently being run. """
            #self.testCCsquare(.5)
            #self.testCsquare(.5)
            self.testLine(1)
        
        def initFile(self, filename):
            """ Write the first line of our outgoing file (variable names). """
            self.filename = filename + ("jerky" if self.jerky else "smooth")
            self.logger.csv(self.filename, ["map_x", "map_y", "reported_x", "reported_y"], folder = "tests")
        
        def logArrival(self, name, x, y):
            self.logger.info("Reached " + str(name) + " at " + str((x,y)))
            self.logger.info("Current pose: " + str((self.navloc.p.x, self.navloc.p.y)))
            self.logger.csv(self.filename, [x, y, self.navloc.p.x, self.navloc.p.y])
        
        def testLine(self, length):
            """ Test behavior with a simple line. 
            
            Args:
                length (float): Length of the desired line (in meters).
            """
            if self.filename is None:
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
            if self.filename is None:
                self.initFile("counterclockwise")
            
            self.testSquare(length, self.cc_square)
        
        def testCsquare(self, length):
            """ Test a clockwise square. 
            
            Args:
                length (float): Length of the desired line (in meters).
            """
            if self.filename is None:
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