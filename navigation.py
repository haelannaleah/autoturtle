""" Local navigation using robot_pose_ekf
    
Author:
    Annaleah Ernst
"""
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from math import atan2, pi

from logger import Logger

class Navigation():
    """ Local navigation.
    
    Attributes:
        p (geometry_msgs.msg.Point): The position of the robot in the odometry frame according to
            the robot_pose_ekf package.
        q (geometry_msgs.msg.Quaternion): The orientation of the robot in the odometry frame
            according the the robot_pose_ekf package.
        angle (float): The angle (in radians) that the robot is from 0. Between -pi and pi
    """
    _HALF_PI = pi / 2.0
    _TWO_PI = 2.0 * pi
    
    def __init__(self):
        self._logger = Logger("Navigation")

        self.p = None
        self.q = None
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

    def _ekfCallback(self, data):
        """ Process robot_pose_ekf data. """
        self.p = data.pose.pose.position
        self.q = data.pose.pose.orientation
        
        # since a quaternion respresents 3d space, and turtlebot motion is in 2d, we can just
        # extract the only non zero euler angle as the angle of rotation in the floor plane
        self.angle = tf.transformations.euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])[-1]

    def goToPosition(self, destination):
        """ Move from current position to desired waypoint in the odomety frame.
            
        Args:
            destination (geometry_msgs.msg.Point): A destination relative to the origin, in meters.
        
        Returns:
            True if we are close to the desired location, 0 if the goal is straight ahead, and the difference 
                between the current angle and the desired angle if we are not on course.
        """
        # take the angle between our position and destination in the odom frame
        turn = atan2(destination.y - self.p.y, destination.x - self.p.x)
        
        # Set the turn angle to behave as the angle that is the minimum distance from our current pose.
        # The closest equivalent angle may be slightly greater than pi or slightly less than -pi, and since
        # our math is always bounded by pi and -pi, we may need to adjust to be the most efficient.
        turn_angle = min([turn, turn + self._TWO_PI, turn - self._TWO_PI], key = lambda t: abs(t - self.angle))

        # we're near our final location
        if np.isclose([self.p.x, self.p.y], [destination.x, destination.y], atol=.05).all():
            return True
        
        # our orientation has gotten off
        elif not np.isclose(self.angle, turn_angle, atol=0.05):
            return self.angle - turn_angle

        # otherwise, move toward our goal
        else:
            return 0

if __name__ == "__main__":
    from tester import Tester
    #from motion import Motion
    from safe_motion import SafeMotion
    
    class NavigationTest(Tester):
        def __init__(self):
            self.navigation = Navigation()
            self.motion = SafeMotion(0)
            
            self.stopping = False
            
            # linear test
            self.reached_goal = False
            
            # square test
            self.reached_corner = [False, False, False, False]
            self.corner_counter = 0
            
            Tester.__init__(self, "Navigation")

        def main(self):
            """ The test currently being run. """
            self.testSquare(1)
            # self.testLine(1)
        
        def gotToPos(self, name, x, y):
            """ Default behavior for testing goToPosition. 
            
            Args:
                name (str): Name of the waypoint we are approaching.
                x (float): The x coordinate of the desired location (in meters from the origin).
                y (float): The y coordinate of the desired location (in meters from the origin).
            """
            nav_val = self.navigation.goToPosition(Point(x,y,0))
            
            # did we reach our waypoint?
            if nav_val is True or self.stopping is True:
                if self.motion.walking or self.motion.turning:
                    self.motion.stop()
                    self.stopping = True
                else:
                    self.logger.info("Reached " + str(name) + " at " + str((x,y)))
                    self.logger.info("Current pose: " + str((self.navigation.p.x, self.navigation.p.y)))
                    self.stopping = False
                    return True
            
            # our goal is straight ahead
            elif nav_val == 0:
                if self.motion.turning:
                    self.motion.stop_rotation(now=True)
                else:
                    self.motion.walk()
            
            # we need to turn to reach our goal
            else:
                if self.motion.walking and abs(nav_val) > pi / 2:
                    self.motion.stop()
                else:
                    self.motion.turn(nav_val < 0, .25 if self.motion.walking else 1)
            
            return False
        
        def testLine(self, length):
            """ Test behavior with a simple line. 
            
            Args:
                length (float): Length of the desired line (in meters).
            """
            if not self.reached_goal:
                self.reached_goal = self.goToPos("end point", length, 0)
            else:
                self.reached_goal = self.goToPos("home", 0, 0)
        
        def testSquare(self, length):
            """ Test behavior with a simple square. 
            
            Args:
                length (float): Length of the sides of the square (in meters).
            """
        
            # test a simple square
            if not self.reached_corner[self.corner_counter]:
                x_coord = length * (self.corner_counter == 1 or self.corner_counter == 2)
                y_coord = length * (self.corner_counter == 2 or self.corner_counter == 3)
                self.reached_corner[self.corner_counter] = self.gotToPos("corner " + str(self.corner_counter), x_coord, y_coord)
            else:
                if self.corner_counter == len(self.reached_corner) - 1:
                    self.reached_corner = [False] * len(self.reached_corner)
                self.corner_counter = (self.corner_counter + 1) % len(self.reached_corner)
    
        def shutdown(self):
            self.motion.shutdown(self.rate)
            Tester.shutdown(self)
        

    NavigationTest()