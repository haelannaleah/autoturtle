import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers

from logger import Logger

class Localization():
    def __init__(self, tag_dict):
        
        # listen to the raw AprilTag data
        self.tags = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._tagCallback, queue_size=1)

    def _tagCallback(self, data):
        """ Extract raw tag data from the ar_pose_marker topic.
        
        Note: Tag data comes in the camera frame, not the map frame.
        """
        if data.markers:
            self.tags = data.markers
        else:
            self.tags = None

if __name__ == "__main__":
    from tester import Tester

    class LocalizationTest(Tester):
        """ Run localization tests. """
        def __init__(self):
            Tester.__init__(self, "Localization")
            
            # set up localization
            self.localization = Localization()

        def main(self):
            self.logger.info(self.localization.tags)