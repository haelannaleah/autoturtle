""" Local navigation using robot_pose_ekf
    
Author:
    Annaleah Ernst
"""
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped

from logger import Logger():

class Navigation():
    """ """
    def __init__(self):
        self._logger = Logger("Navigation")

        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

    def _ekfCallback(self, data):
        self._logger.debug(data)

if __name__ == "__main__":
    from tester import Tester

    class NavigationTest(Tester):
        def __init__(self):
            Tester.__init__(self, "Navigation")

        def main(self):
            pass