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
            self._transformPos()
        else:
            self.tags = {}

    def _transformPos(self):
        """ Attempt a frame transformation. """
        for id in self.tags:
            try:
                base_frame = self._tf_listener.transformPose('/base_footprint',  self.tags[id])
                self._logger.info(base_frame)
                self._logger.info(tf.transformations.euler_from_quaternion([base_frame.pose.orientation.x, base_frame.pose.orientation.y, base_frame.pose.orientation.z, base_frame.pose.orientation.w]))
                self._logger.info(self._tf_listener.transformPose('/odom',  self.tags[id]))
            except Exception as e:
                self._logger.error(e)

if __name__ == "__main__":
    from tester import Tester

    class LocalizationTest(Tester):
        """ Run localization tests. """
        def __init__(self):
            Tester.__init__(self, "Localization")
            
            # set up localization
            self.localization = Localization()

        def main(self):
            pass

    LocalizationTest().run()