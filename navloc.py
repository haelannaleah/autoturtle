""" Navigation and localization
    
Author:
    Annaleah Ernst
"""
import tf
import rospy

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from math import sin, cos

from navigation import Navigation
from localization import Localization
from logger import Logger

class NavLoc(Navigation, Localization):
    def __init__(self, landmarks):
        
        # create transformation object
        self._transform = {"position": Point(0,0,0), "angle" = 0}
    
        # initialize what we're inheriting from
        Navigation.__init__(self)
        Localization.__init__(self, landmarks)
        self._raw_pose = Pose()

        self._logger = Logger("NavLoc")
    
    def _estimatePose(self):
        """ Override pose estimation to include pose calculation. """
    
        Localization()._estimatePose(self)
        
        # if there is currently no estimated pose, nothing more to do here
        if self.estimated_pose is None:
            return

        ekf_pose = deepcopy(self._raw_pose)
        ekf_q = ekf_pose.orientation
        ekf_angle = tf.transformations.euler_from_quaternion([ekf_q.x, ekf_q.y, ekf_q.z, ekf_q.w])[-1]
        
        self._transform["angle"] = self.estimated_angle - ekf_angle
        self._transform["position"].x = self.estimated_pose.x - ekf_pose.position.x
        self._transform["position"].y = self.estimated_pose.y - ekf_pose.position.y

    def _ekfCallback(self, data):
        """ Process robot_pose_ekf data. """
        self._raw_pose = data.pose.pose
        p = self._raw_pose.position
        q = self._raw_pose.orientation

        # transform from odom to the map frame
        self.p = Point()
        self.p.x = self._transform["position"].x + p.x * cos(self._transform["angle"]) + p.y * sin(self._transform["angle"])
        self.p.y = self._transform["position"].y - p.x * sin(self._transform["angle"]) + p.y * cos(self._transform["angle"])
        
        # since a quaternion respresents 3d space, and turtlebot motion is in 2d, we can just
        #   extract the only non zero euler angle as the angle of rotation in the floor plane
        self.angle = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1]
        self.angle += self.transform["angle"]

        # we're deciding not to care about the quaternion for now
        self.q = None

