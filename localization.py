""" Global localization.
    
Author:
    Annaleah Ernst
"""
import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped

from logger import Logger

class Localization():
    """ Handle landmark detection and global localization.
    
    Attributes:
        tags (geometry_msgs.msg.PoseStamped dict): A dict of all the AprilTags currently in view.
            Note: in the /ar_marker_<id> frame.
        landmarks_relative (geometry_msgs.msg.PoseStamped dict): Same as above, but in the robot base
            frame.
        landmarks_odom (geometry_msgs.msg.PoseStamped dict): Same as above, but in the odom frame.
    """
    def __init__(self):
        self._logger = Logger("Localization")
        
        # listen to the raw AprilTag data
        self.tags = {}
        self.landmarks_relative = {}
        self.landmarks_odom = {}
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._tagCallback, queue_size=1)
    
        # listen for frame transformations
        self._tf_listener = tf.TransformListener()

    def _tagCallback(self, data):
        """ Extract raw tag data from the ar_pose_marker topic.
        
        Note: Raw tag data comes in the /ar_marker_<id> frame frame, not the map frame.
        """
        if data.markers:
            # convert marker data into PoseStamped
            for marker in data.markers:
                self.tags[marker.id] = PoseStamped(marker.header, marker.pose.pose)
                
                # set the frame of the data
                self.tags[marker.id].header.frame_id = '/ar_marker_' + str(marker.id)
                
                # set the time to show that we only care about the most recent available transform
                self.tags[marker.id].header.stamp = rospy.Time(0)
            
            self.landmarks_relative = self._transformTags('/base_footprint')
            self.landmarks_odom = self._transformTags('/odom')
        else:
            self.tags = {}
            self.landmarks_relative = {}
            self.landmarks_odom = {}

    def _transformTags(self, target_frame):
        """ Convert all of the visible tags to target frame.
        
        Args:
            target_frame (string): The desired final coordinate frame.
            
        Returns:
            A geometry_msgs.msg.PoseStamped dictionary containing the positions in the target frame
                of the visible AprilTags (contingent on successful transformation).
        """
        transformed = {id : self._transformPose(target_frame, self.tags[id]) for id in self.tags}
        return {id : transformed[id] for id in transformed if transformed[id] is not None}

    def _transformPose(self, target_frame, marker):
        """ Attempt a frame transformation. 
        
        Args:
            target_frame (string): The desired final coordinate frame.
            marker (geometry_msgs.msg.PoseStamped): The pose we wish to transform.
        
        Returns:
            A transformed geometry_msgs.msg.PoseStamped object if the transformation was successful,    
                None otherwise.
        """
        try:
            return self._tf_listener.transformPose(target_frame,  marker)
        except Exception as e:
            self._logger.error(e)
        
        return None

if __name__ == "__main__":
    from tester import Tester
    from math import degrees
    from copy import deepcopy

    class LocalizationTest(Tester):
        """ Run localization tests. """
        def __init__(self):
            Tester.__init__(self, "Localization")
            
            # set up localization
            self.localization = Localization()

        def main(self):
            """ Run main tests. """
            self.logger.info("orientation")
            self.logOrientation(self.localization.landmarks_relative)
            self.logPosition(self.localization.landmarks_relative)
            self.logger.info("odom")
            self.logOrientation(self.localization.landmarks_odom)
            self.logPosition(self.localization.landmarks_relative)
            
        def logPosition(self, incoming_landmarks):
            """ Print the position of landmarks in meters. """
            landmarks = deepcopy(incoming_landmarks)
            for id in landmarks:
                self.logger.debug("\n" + str(landmarks[id].pose.position), var_name = id)
        
        def logOrientation(self, incoming_landmarks):
            """ Print the orientation of landmarks as a Euler Angle in degrees. """
            landmarks = deepcopy(incoming_landmarks)
            for id in landmarks:
                q = landmarks[id].pose.orientation
                self.logger.debug([round(degrees(t)) for t in tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])], var_name = id)

    LocalizationTest().run()