""" Local navigation using robot_pose_ekf
    
Author:
    Annaleah Ernst
"""
import rospy
import tf

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

        self.p = None
        self.q = None
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

    def _ekfCallback(self, data):
        self.p = data.pose.pose.position
        self.q = data.pose.pose.orientation
        
        # since a quaternion respresents 3d space, and turtlebot motion is in 2d, we can just
        # extract the only non zero euler angle as the angle of rotation in the floor plane
        self.angle = tf.transformations.euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])[-1]

    def wrapTurnAngle(self, turn):
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

if __name__ == "__main__":
    from tester import Tester

    class NavigationTest(Tester):
        def __init__(self):
            self.navigation = Navigation()
            Tester.__init__(self, "Navigation")

        def main(self):
            self.logger.debug(self.navigation.angle, var_name = "cur_angle")
            self.logger.debug(self.navigation.wrapTurnAngle(-2), var_name="-2")
            self.logger.debug(self.navigation.wrapTurnAngle(2), var_name="2")

    NavigationTest()