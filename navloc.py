""" Navigation and localization
    
Author:
    Annaleah Ernst
"""
import tf
import rospy
import threading

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion

from navigation import Navigation
from localization import Localization
from logger import Logger

class NavLoc(Navigation, Localization):
    def __init__(self, landmarks):
        
        # create transformation and broadcaster
        self._broadcaster = tf.TransformBroadcaster()
        self._transform = {"position": Point(0,0,0), "angle" = 0}
        self._lock = threading.lock()
        self._tf_thread = threading.Thread(target=self._broadcastTransform)
        self._tf_thread.start()
    
        # initialize what we're inheriting from
        Navigation.__init__(self)
        Localization.__init__(self, landmarks)
        self._raw_pose = Pose

        self._logger = Logger("NavLoc")

    def _broadcastTransform(self):
        """ Broadcast the current transformation. """
        
        # set the broadcast rate
        rate = rospy.Rate(10)
        
        # as long as the robot is going, broadcast the transformation
        while not rospy.is_shutdown:
            # TODO: determine if locking is necessary
            self._lock.acquire()
            self._broadcaster(self._transform["position"], self._transform["orientation"],
                rospy.Time.now(), "/robot_pose_ekf", "/map")
            self._lock.release()
            
            rate.sleep()
    
    def _estimatePose(self):
        """ Override pose estimation to include pose calculation. """
    
        Localization()._estimatePose(self)
        
        # if there is currently no estimated pose, nothing more to do here
        if self.estimated_pose is None:
            return

        ekf_pose = deepcopy(self._raw_pose)
        ekf_q = ekf_pose.orientation
        ekf_angle = tf.transformations.euler_from_quaternion([ekf_q.x, ekf_q.y, ekf_q.z, ekf_q.w])[-1]
        
        self.angle = self.estimated_pose.

    def _ekfCallback(self, data):
        """ Process robot_pose_ekf data. """
        self._raw_pose = data.pose.pose

        # transform to the map frame
        self.p = None
        self.q = None
        
        # since a quaternion respresents 3d space, and turtlebot motion is in 2d, we can just
        #   extract the only non zero euler angle as the angle of rotation in the floor plane
        self.angle = tf.transformations.euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])[-1]

