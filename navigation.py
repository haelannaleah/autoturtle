""" Local navigation using robot_pose_ekf
    
Author:
    Annaleah Ernst
"""
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from math import pi

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

    def wrapAngle(self, turn):
        """ Wrap around the pi to back to -pi, if necessary
    
        If the angles are in adjacent quadrants where the angles wrap around (since we go from -pi to pi),
            we need to make sure that they stil treat each other like adjacent quadrants when we're trying
            to stick to a course.
        
        Args:
            turn (float): The desired orientation of the robot in radians to achieve our goal.
            
        Ret:
            The angle of the orientation accounting for wrap around.
        """
        if self.angle > self._HALF_PI and turn < -self._HALF_PI:
            return turn + self._TWO_PI
            
        if turn > self._HALF_PI and self.angle < -self._HALF_PI:
            return turn - self._TWO_PI

        return turn

    def goToPosition(self, destination):
        """ Move from current position to desired waypoint.
            
        Args:
            destination (geometry_msgs.msg.Point): A destination relative to the origin.
        
        Returns:
            True if we are close to the desired location, False otherwise.
        """
        turn_angle = self.wrapAngle(atan2(destination.y - self.p.y, destination.x - self.p.x))

        if np.isclose([self.p.x, self.p.y], [destination.x, destination.y], atol=.05).all():
            return True
        
        elif not np.isclose(self.angle, turn_angle, atol=0.15):
            self.motion.turn(cur_orientation < desired_turn, .5)

        else:
            self.motion.walk()

        return False

if __name__ == "__main__":
    from tester import Tester

    class NavigationTest(Tester):
        def __init__(self):
            self.navigation = Navigation()
            self.reached_goal = False
            
            Tester.__init__(self, "Navigation")

        def main(self):
        
            # test a simple line
            if not self.reached_goal:
                if self.navigation.goToPosition(Point(.5,0,0)):
                    self.logger.info("Reached goal point!!")
                    self.reached_goal = True
            else:
                if self.navigation.goToPosition(Point(0,0,0)):
                    self.logger.info("Returned home")
                    reached_goal = False

    NavigationTest()