""" Local navigation using robot_pose_ekf
    
Author:
    Annaleah Ernst
"""
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PointStamped
from math import atan2, pi, sqrt
from random import choice
from std_msgs.msg import Empty
from time import time

from logger import Logger
from motion import Motion
from sensors import Sensors
from tf_transformer import TfTransformer

class Navigation(Motion, TfTransformer):
    """ Local navigation.
    
    Args:
        jerky (bool, optional): If true, robot will not decelerate, but stop abruptly.
            Defaults to False.
        walking_speed (float, optional): Percentage of maximum speed, magnitude between 0 and 1.
                Values with magnitude greater than 1 will be ignored.
    
    Attributes:
        p (geometry_msgs.msg.Point): The position of the robot in the ekf odometry frame according to
            the robot_pose_ekf package.
        q (geometry_msgs.msg.Quaternion): The orientation of the robot in the ekf odometry frame
            according the the robot_pose_ekf package.
        angle (float): The angle (in radians) that the robot is from 0 in the ekf odometry frame. 
            Between -pi and pi
    """
    # avoid recomputing constants
    _HALF_PI = pi / 2.0
    _TWO_PI = 2.0 * pi
    _PI_OVER_FOUR = pi / 4.0
    
    # create thresholds
    _MIN_STATIONARY_TURN_SPEED = 0.5
    _MIN_MOVING_TURN_SPEED = 0.15
    _MAX_MOVING_TURN = pi / 6
    _MIN_LINEAR_SPEED = .25
    
    # set avoidance data
    _AVOID_BUMP_TURN = pi / 6
    _AVOID_TURN = .2
    _AVOID_DIST = 0.25
    
    def __init__(self, jerky = False, walking_speed = 1):
    
        # listen for frame transformations
        TfTransformer.__init__(self)
    
        # initialize motion component of navigation
        self._motion = Motion()
        self._sensors = Sensors()
        self._jerky = jerky
        self._walking_speed = min(abs(walking_speed), 1)
        self._logger = Logger("Navigation")
        
        # set up obstacle avoidance
        self._avoiding = False
        self._obstacle = False
        self._bumped = False
        
        # we're going to send the turtlebot to a point a quarter meter ahead of itself
        self._avoid_goto = PointStamped()
        self._avoid_goto.header.frame_id = "/base_footprint"
        self._avoid_goto.point.x = self._AVOID_DIST
        self._avoid_target = None
        self._avoid_turn = None

        # subscibe to the robot_pose_ekf odometry information
        self.p = None
        self.q = None
        self.angle = 0
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)
        
        # set up navigation to destination data
        self._reached_goal = True
    
        # set up the odometry reset publisher (publishing Empty messages here will reset odom)
        reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1)
        
        # reset odometry (these messages take about a second to get through)
        timer = time()
        while time() - timer < 1 or self.p is None:
            reset_odom.publish(Empty())

    def _ekfCallback(self, data):
        """ Process robot_pose_ekf data. """
        
        # get the direct data
        self.p = data.pose.pose.position
        self.q = data.pose.pose.orientation
        
        # since a quaternion respresents 3d space, and turtlebot motion is in 2d, we can just
        #   extract the only non zero euler angle as the angle of rotation in the floor plane
        self.angle = tf.transformations.euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])[-1]

    def _getDestData(self, dest_x, dest_y):
        """ Move from current position to desired waypoint in the odomety frame.
            
        Args:
            dest_x: Distance from the x origin in meters.
            dest_y: Distance from the y origin in meters.
        
        Returns:
            True if we are close to the desired location 
            0 if the goal is straight ahead
            The difference between the current angle and the desired angle if we are not on course.
                A negative value indicates that the desired angle is that many radians to the left of 
                the current orientation, positive indicates the desired angle is to the right.
        """
        # take the angle between our position and destination in the ekf odom_combined frame
        turn = atan2(dest_y - self.p.y, dest_x - self.p.x)
        
        # set the turn angle to behave as the angle that is the minimum distance from our current pose
        turn_angle = self._wrapAngle(turn)

        # we're (pretty) near our final location
        if np.allclose([self.p.x, self.p.y], [dest_x, dest_y], atol=.05):
            return True
        
        # our orientation has gotten off and we need to adjust
        elif not np.isclose(self.angle, turn_angle, atol=0.05):
            return self.angle - turn_angle

        # otherwise, move toward our goal
        else:
            return 0

    def _handleObstacle(self, turn_delta):
        """ Take stock of sensor data when deciding how to move. """
    
        # if we see a cliff or get picked up, stop
        if self._sensors.cliff or self._sensors.wheeldrop:
            self._motion.stop(now=True)
    
        # if we hit something, stop
        elif self._sensors.bump:
            self._bumped = True
        
            # stop if we hit something
            if self._motion.walking:
                self._motion.stopLinear(now = True)

            # turn away from what we hit
            self._motion.turn(self._sensors.bumper > 0, speed = self._MIN_STATIONARY_TURN_SPEED)

        # if we've been bumped, turn away!
        elif self._bumped:
        
            # set the post bump turn
            if self._avoid_turn is None:
                self._avoid_turn = self.angle + self._AVOID_BUMP_TURN * self._motion.turn_dir
        
            # turn until we reach the appropriate bump angle
            if self._goToOrient(self.angle - self._wrapAngle(self._avoid_turn)):
                self._avoid_turn = None
                self._bumped = False
                self._avoiding = True
        
        # if we've just reached a goal, no reason to stop and turn unnecessarily
        elif self._reached_goal and self._motion.stopping:
            self._logger.debug("in stop")
            return False

        # no colliding with anything
        elif self._sensors.obstacle:
            
            # stop so we don't hit anything
            if self._motion.walking:
                self._motion.stopLinear()
        
            # we're already turning in the right direction to not hit anything
            elif turn_delta != 0 and (self._sensors.obstacle_dir > 0 == turn_delta < 0):
                self._logger.debug("same turn")
                self._obstacle = False
                self._avoiding = False
                return False
                
            # make sure that we turn away from the obstacle
            elif not self._obstacle:
                self._motion.stopRotation(now = True)
                self._obstacle = True
                self._avoiding = True
                self._logger.debug("obstacling")
                
            # turn away from the obstacle if necessary
            else:
                self._motion.turn(self._sensors.obstacle_dir > 0)
                self._logger.debug("turning away")
            
        # otherwise, we go into avoidance mode
        elif self._avoiding:
            
            # if we encounter a new obstacle, we want to turn in the right way
            self._obstacle = False
            
            # if we see a wall while we're avoiding, we should deal with it
            if self._sensors.wall:
            
                # turn away from the wall
                self._motion.turn(self._sensors.wall_dir > 0, speed = self._MAX_MOVING_TURN)
                
                # set avoidance behavior
                self._avoid_turn = self.angle + self._AVOID_TURN * self._motion.turn_dir
                self._avoid_target = None
            
            elif self._avoid_turn is not None:
            
                # turn in the prescribed avoidance direction
                if self._goToOrient(self.angle - self._wrapAngle(self._avoid_turn)):
                    self._avoid_turn = None
        
            elif self._avoid_target is None:
            
                # get the most recent transformation to set target in the odom frame
                self._avoid_goto.header.stamp = rospy.Time(0)
                self._avoid_target = self._tf_listener.transformPoint("/odom", self._avoid_goto)
            
            else:
            
                # go to the avoidance way point
                if self._goToPos(self._getDestData(self._avoid_target.point.x, self._avoid_target.point.y)):
                    self._avoiding = False
                    self._avoid_target = None
                    return False
        else:
            return False

        return True
    
    def _goToOrient(self, turn_delta):
        """ Turn in the direction of the turn delta. """
        
        if np.isclose(turn_delta, 0, atol=0.05):
            # we're facing the direction we're supposed to
            return True
    
        # make sure we're turning in the correct direction, and stop the turn if we're not
        if (turn_delta <= 0) != (self._motion.turn_dir >= 0):
            self._motion.stopRotation(now = True)

        # don't allow ourselve to spin in circles
        if self._motion.walking and abs(turn_delta) > self._MAX_MOVING_TURN:
            self._motion.stopLinear(now = self._jerky)
        
        # don't awkwardly stall at a low speed
        if self._motion.starting:
            self._motion.walk(speed=self._walking_speed)
        
        # set the differential turn speed
        differential_turn = (turn_delta / self._HALF_PI)**2
        
        # perform our turn with awareness how far off the target direction we are
        self._motion.turn(turn_delta < 0,  differential_turn + (self._MIN_MOVING_TURN_SPEED if self._motion.walking else self._MIN_STATIONARY_TURN_SPEED))

        return False

    def _goToPos(self, turn_delta):
        """ Go to a position in the odometry frame. """
        
        # otherwise, did we reach our waypoint?
        if turn_delta is True or self._reached_goal is True:
        
            # we've reached a waypoint, but we may still need to stop
            self._reached_goal = True
            
            # give ourselves a moment to stop if we're not in jerky mode
            if self._motion.walking or self._motion.turning:
                self._motion.stop(now = self._jerky)
            
            # let the user know that we made it!
            else:
                self._reached_goal = False
                return True
        
        # our goal is straight ahead
        elif turn_delta == 0:
        
            # if we're turning, we need to stop
            if self._motion.turning:
                self._motion.stopRotation(now = True)
        
            # onwards we go at the desired pace
            self._motion.walk(speed = self._walking_speed)
        
        # we need to turn to reach our goal
        else:
            self._goToOrient(turn_delta)

        # we're still moving towards our goal (or our stopping point), or we've gotten trapped
        return False
        
    def _wrapAngle(self, angle):
        """ Set the turn angle to behave as the angle that is the minimum distance from our current pose.
        
        Note:
            The closest equivalent angle may be slightly greater than pi or slightly less than -pi; we want
                angles at pi and -pi to behave as if they are right next to each other, so we may need to wrap
                around by adding or subtracting two pi.
        """
        return min([angle, angle + self._TWO_PI, angle - self._TWO_PI], key = lambda a: abs(a - self.angle))
        
    def goToOrientation(self, angle):
        """ Go to orientation in the odometry frame. """
        
        # get the closest equivalent angle to our current pose
        turn_angle = self._wrapAngle(angle)
    
        return self._goToOrient(self.angle - turn_angle)

    def goToPosition(self, x, y):
        """ Go to waypoint in the odometry frame.
        
        Args:
            x (float): The x coordinate of the desired location (in meters from the origin).
            y (float): The y coordinate of the desired location (in meters from the origin).
        """
        # if we've encountered some sort of obstacle, we haven't even tried to get to the current position
        turn_delta = self._getDestData(x, y)
        
        if self._handleObstacle(turn_delta):
            return False

        return self._goToPos(turn_delta)
        
    def csvLogArrival(self, test_name, x, y, folder = "tests"):
        """ Log Turtlebot's arrival at a waypoint. """
        
        # send data to the csv logger
        self._logger.csv(test_name + "_waypoints", ["X_target", "Y_target", "X_reported", "Y_reported"],
                            [x, y, self.p.x, self.p.y], folder = folder)
    
    def csvLogEKF(self, test_name, folder = "tests"):
        """ Log the current turtlebot pose information. """
        
        # create csv dict and log data
        self._logger.csv(test_name + "_ekf", ["X", "Y", "qZ", "qW", "yaw"],
                            [self.p.x, self.p.y, self.q.z, self.q.w, self.angle], folder = folder)
    
    def shutdown(self, rate):
        """ Stop the turtlebot. """
        
        self._motion.shutdown(rate)
        self._logger.shutdown()

if __name__ == "__main__":
    from tester import Tester
    
    class NavigationTest(Tester):
        """ Run local navigation tests. """
        def __init__(self):
            Tester.__init__(self, "Navigation")

            # tests to run:
            #   square with Motion module, minimal.launch
            #   square with Motion module, navigation launch
            # expect all to turn out the same, but need to sanity check
            #self.motion = Motion()
            
            # flag for a jerky stop
            self.jerky = True
            
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
            self.filename = None
        
            self.navigation = Navigation(self.jerky)

        def main(self):
            """ The test currently being run. """
            #self.testCCsquare(1)
            self.testCsquare(1)
            #self.testLine(1.5)
            self.navigation.csvLogEKF(self.filename)
        
        def initFile(self, filename):
            """ Write the first line of our outgoing file (variable names). """
            self.filename = filename + ("_jerky" if self.jerky else "_smooth")
        
        def testLine(self, length):
            """ Test behavior with a simple line. 
            
            Args:
                length (float): Length of the desired line (in meters).
            """
            self.initFile("line")
            
            if not self.reached_corner[0]:
                self.reached_corner[0] = self.navigation.goToPosition(0, 0)
                if self.reached_corner[0]:
                    self.logger.debug("reached corner 0")
                    self.navigation.csvLogArrival(self.filename, 0, 0)
        
            elif self.navigation.goToPosition(length, 0):
                self.reached_corner[0] = False
                self.navigation.csvLogArrival(self.filename, length, 0)
                self.logger.debug("reached corner 1")
    
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
                self.reached_corner[self.corner_counter] = self.navigation.goToPosition(corners[self.corner_counter][0]*length, corners[self.corner_counter][1]*length)
            
            else:
                self.navigation.csvLogArrival(self.filename, corners[self.corner_counter][0]*length, corners[self.corner_counter][1]*length)
                if self.corner_counter == len(self.reached_corner) - 1:
                    self.reached_corner = [False] * len(self.reached_corner)
                self.corner_counter = (self.corner_counter + 1) % len(self.reached_corner)
    
        def shutdown(self):
            """ Kill all behavioral test processes. """
            self.navigation.shutdown(self.rate)
            Tester.shutdown(self)
        
    NavigationTest().run()