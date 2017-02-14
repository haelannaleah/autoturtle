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
            Note: in the /camera_rgb_optical_frame.
    """
    def __init__(self):
        self._logger = Logger("Localization")
        
        # listen to the raw AprilTag data
        self.tags = {}
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._tagCallback, queue_size=1)
    
        # listen for frame transformations
        self._tf_listener = tf.TransformListener()

    def _tagCallback(self, data):
        """ Extract raw tag data from the ar_pose_marker topic.
        
        Note: Tag data comes in the camera frame, not the map frame.
        """
        if data.markers:
            self.tags = {marker.id : PoseStamped(marker.header, marker.pose.pose) for marker in data.markers}
            self.landmarks_relative = {id : self._transformPos('/base_footprint', self.tags[id]) for id in self.tags}
            self.landmarks_odom = {id : self._transformPos('/odom', self.tags[id]) for id in self.tags}
        else:
            self.tags = {}
            self.landmarks_relative = {}
            self.landmarks_odom = {}

    def _transformPos(self, target_frame, marker):
        """ Attempt a frame transformation. 
        
        Args:
            target_frame (string): The desired final coordinate frame.
            marker
        """
        try:
            return self._tf_listener.transformPose(target_frame,  marker)
        except Exception as e:
            self._logger.error(e)
        
        return None

if __name__ == "__main__":
    from tester import Tester

    class LocalizationTest(Tester):
        """ Run localization tests. """
        def __init__(self):
            Tester.__init__(self, "Localization")
            
            # set up localization
            self.localization = Localization()

        def main(self):
            # print some ids
            for id in self.localization.landmarks_relative:
                q = self.localization.landmarks_relative.pose.orientation
                self.logger.info(id)
                self.logger.info(tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]))

    LocalizationTest().run()