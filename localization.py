""" Global localization.
    
Author:
    Annaleah Ernst
"""
import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, QuaternionStamped

from logger import Logger

class Localization():
    """ Handle landmark detection and global localization.
    
    Attributes:
        tags (geometry_msgs.msg.PoseStamped dict): A dict of all the AprilTags currently in view in their raw form.
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
        """ Extract and process tag data from the ar_pose_marker topic. """
        if data.markers:
            # use a list comprehension to convert the raw marker data into a dictionary of PoseStamped objects
            #   I promise, its less scary than it looks...
            self.tags = {marker.id : PoseStamped(marker.header, marker.pose.pose) for marker in data.markers}
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
                of the visible AprilTags that were successfully transformed.
                
        Note: 
            Raw tag orientation data comes in the /ar_marker_<id> frame, and its position data come in the
                /camera_rgb_optical_frame, so our transformations must reflect this.
        """
        transformed = {}
        for id in self.tags:
            # copy the header from the current tag
            header = self.tags[id].header
            
            # set the time to show that we only care about the most recent available transform
            self.tags[id].header.stamp = rospy.Time(0)
            
            # orientation data starts in the ar_marker_<id> frame; we need to switch the header frame for
            #   the next transformation to reflect this
            header.frame_id = '/ar_marker_' + str(id)
            try:
                orientation = self._tf_listener.transformQuaternion(target_frame, QuaternionStamped(header, self.tags[id].pose.orientation))
            except Exception as e:
                # something went wrong with the coordinate transform, so we should move along
                self._logger.error(e)
                continue
                
            # incoming position data is relative to the rgb camera frame, so we just create a PointStamped with the
            #   correct header and tag position
            header.frame_id = '/camera_rgb_optical_frame'
            try:
                position = self._tf_listener.transformPoint(target_frame, PointStamped(header, self.tags[id].pose.position))
            except Exception as e:
                # something went wrong with the coordinate transform, so we should move along
                self._logger.error(e)
                continue
            
            # if we made it this far, then we can add our pose data to our dictionary!
            transformed[id] = PoseStamped(position.header, Pose(position.point, orientation.quaternion))
            
        return transformed

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
            self.logPosition(self.localization.landmarks_odom)
            
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