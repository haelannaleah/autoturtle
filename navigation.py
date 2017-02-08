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
from motion import Motion

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
        
        self.motion = Motion()

        self.p = None
        self.q = None
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

    def _ekfCallback(self, data):
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
            True if we are close to the desired location, False otherwise.
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
        elif not np.isclose(self.angle, turn_angle, atol=0.15):
            if self.motion.walking:
                self.motion.stop_linear()
            else:
                self.motion.turn(self.angle < turn_angle, .5)

        # otherwise, move toward our goal
        else:
            if self.motion.turning:
                self.motion.stop_rotation()
            else:
                self.motion.walk()

        return False

    def shutdown(self, rate):
        """ Bring the robot to a gentle stop. 
        
        Args:
            rate (rospy.Rate): The refresh rate of the enclosing module.
        """
        self.motion.shutdown(rate)

if __name__ == "__main__":
    from tester import Tester

    class NavigationTest(Tester):
        def __init__(self):
            self.navigation = Navigation()
            
            # linear test
            self.reached_goal = False
            
            # square test
            self.reached_corner = [False, False, False]
            
            Tester.__init__(self, "Navigation")

        def main(self):
            """ The test currently being run. """
            self.testSquare()
        
        def gotToPos(self, name, x, y):
            """ Default behavior for testing goToPosition. """
            if self.navigation.goToPosition(Point(x,y,0)):
                self.logger.info("Reached " + str(name) + " at " + str((x,y)))
                self.logger.info("Current pose: " + str((self.navigation.p.x, self.navigation.p.y)))
                return True
            else:
                return False
        
        def testLine(self):
            """ Test behavior with a simple line. """
            if not self.reached_goal:
                self.reached_goal = self.goToPos("end point", 1, 0)
            else:
                self.reached_goal = self.goToPos("home", 0, 0)
        
        def testSquare(self):
            """ Test behavior with a simple square. """
        
            # test a simple square
            if not self.reached_corner[0]:
                self.reached_corner[0] = self.gotToPos("corner 1", 1, 0)
        
            elif not self.reached_corner[1]:
                self.reached_corner[1] = self.gotToPos("corner 2", 1, 1)
                    
            elif not self.reached_corner[2]:
                self.reached_corner[2] = self.gotToPos("corner 3", 0, 1)
        
            else:
                if self.gotToPos("corner 0", 0, 0):
                    self.reached_corner = [False] * len(self.reached_corner)
    
        def shutdown(self):
            self.navigation.shutdown(self.rate)
            Tester.shutdown(self)
        

    NavigationTest()