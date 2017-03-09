""" Navigation and localization
    
Author:
    Annaleah Ernst
"""
import tf
import rospy
import threading

from geometry_msgs.msg import Pose, Point, Quaternion

from navigation import Navigation
from localization import Localization
from logger import Logger

class NavLoc(Navigation, Localization):
    def __init__(self, landmarks):
        
        # create transformation and broadcaster
        self._broadcaster = tf.TransformBroadcaster()
        self._transform = {"position": [0,0,0], "orientation" = [0,0,0,1]}
        self._lock = threading.lock()
        self._tf_thread = threading.Thread(target=self._broadcastTransform)
        self._tf_thread.start()
    
        # initialize what we're inheriting from
        Navigation.__init__(self)
        Localization.__init__(self, landmarks)

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

        if self.estimated_pose is None:
            return