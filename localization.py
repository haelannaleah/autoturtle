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
        tags (geometry_msgs.msg.PoseStamped dict): A dict of all the AprilTags currently in view in 
            their raw form.
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
    
    def _attemptLookup(self, transform_func, target_frame, object):
        """ Attempt a coordinate frame transformation.
        
        Args:
            transform_func (tf.TransformListener() function): A transformation function from the tf module.
            target_frame (string): The desired final coordinate frame.
            object (PoseStamped, PointStamped, QuaternionStamped): A stamped object to be transformed.
            
        Returns:
            An object transformed into the correct frame if successful, None otherwise.
        """
        try:
            # attempt transformation
            return transform_func(target_frame, object)
        
        except tf.ExtrapolationException as e:
            # we're trying to get a transformation that's not current
            self._logger.warn(e)
            
        except tf.LookupException as e:
            # the transformations aren't being published
            self._logger.error(str(e) + "Is the mobile base powered on? Has the Turtlebot been brought online?")
        
        except Exception as e:
            # something else went wrong
            self._logger.error(e)
        
        # the transformation failed
        return None

    def _tagCallback(self, data):
        """ Extract and process tag data from the ar_pose_marker topic. """
        if data.markers:
            # use a list comprehension to convert the raw marker data into a dictionary of PoseStamped objects
            #   I promise, its less scary than it looks...
            self.tags = {marker.id : PoseStamped(marker.header, marker.pose.pose) for marker in data.markers}
            self.landmarks_relative = self._transformTags('/base_footprint')
            self.landmarks_odom = self._transformTags('/odom')
        else:
            # we don't see any tags, so empty things out
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
            Also note that this is the scary function...
        """
        transformed = {}
        for id in self.tags:
            # get the header from the current tag
            header = self.tags[id].header
            
            # set the time to show that we only care about the most recent available transform
            self.tags[id].header.stamp = rospy.Time(0)
            
            # orientation data is in the ar_marker_<id> frame, so we need to update the starting frame
            #   (if we just transform from the optical frame, then turning the AR tag upside down affects the
            #   reported orientation)
            header.frame_id = '/ar_marker_' + str(id)
            orientation = self._attemptLookup(self._tf_listener.transformQuaternion, \
                            target_frame, QuaternionStamped(header, self.tags[id].pose.orientation))
            
            # make sure the look-up succeeded
            if orientation is None:
                continue
                
            # incoming position data is relative to the rgb camera frame, so we reset the header to the optical
            #   frame to get the correct position (note that this step is necessary since we're getting a shallow
            #   copy of the header)
            header.frame_id = '/camera_rgb_optical_frame'
            position = self._attemptLookup(self._tf_listener.transformPoint, \
                         target_frame, PointStamped(header, self.tags[id].pose.position))
                         
            # make sure the look-up succeeded
            if position is None:
                continue
            
            # if we made it this far, then we can add our pose data to our dictionary!
            transformed[id] = PoseStamped(position.header, Pose(position.point, orientation.quaternion))
            
        return transformed

if __name__ == "__main__":
    import numpy as np
    from tester import Tester
    from math import degrees
    from copy import deepcopy

    class LocalizationTest(Tester):
        """ Run localization tests. """
        def __init__(self):
            Tester.__init__(self, "Localization")
            
            # set up localization
            self.localization = Localization()
            
            self.prev = {}
    
            fields = ["px", "py", "pz", "qx", "qy", "qz", "qw", "r", "p", "y"]
            
            self.csvfields = []
            self.csvfields.append(["raw_" + field for field in fields])
            self.csvfields.append(["odom_" + field for field in fields])
            self.csvfields.append(["relative_" + field for field in fields])
            self.csvtestname = "stationary"

        def main(self):
            """ Run main tests. """
            self.logData()
        
        def logData(self):
            """ Log CSV file and output data to screen. """
            
            # make sure the tags don't go changing on us
            tags = deepcopy(self.localization.tags)
            landmarks_odom = deepcopy(self.localization.landmarks_odom)
            landmarks_relative = deepcopy(self.localization.landmarks_relative)
            
            # separately log all tag data
            for id in tags:
            
                # make sure that the landmark data is in
                if id in landmarks_odom and id in landmarks_relative:
                
                    # convert landmark into csv data
                    logdata = []
                    logdata.append(tags[id])
                    logdata.append(landmarks_odom[id])
                    logdata.append(landmarks_relative[id])
    
                    # if we've never encountered this marker before, or it's values have changed
                    if id not in self.prev or not np.isclose(logdata, self.prevs):
                        self.prev[id] = logdata
                        test_name = self.csvtestname + "_marker" + str(id)
                        
                        # if we've never encountered this marker before, open a new csv file
                        if not self.logger.isLogging(test_name):
                            self.logger.csv(test_name, self.csvfields, folder = "tests")
                    
                        # write data to the csv file
                        self.logger.csv(test_name, logdata, folder = "tests")

                        # log data to the screen as well
                        self.screenLog(tags[id])
                        self.screenLog(landmarks_odom[id])
                        self.screenLog(landmarks_relative[id])
        
        def screenLog(self, landmark, id):
            """ Nicely parse landmarks into easily logable data. """
            self.logger.info("Frame: " + str(landmark.header.frame_id))
            self.logOrientation(landmark, id)
            self.logPosition(landmark, id)
    
        def convertPose(self, landmark):
            p = landmark.pose.position
            q = landmark.pose.orientation
            r, p, y = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            return [p.x, p.y, p.z, q.x, q.y, q.z, q.w, r, p, y]
            
        def logPosition(self, incoming_landmark, id):
            """ Print the position of landmarks in meters. """
            landmark = deepcopy(incoming_landmark)
            self.logger.debug("\n" + str(landmark.pose.position), var_name = "position" + str(id))
        
        def logOrientation(self, incoming_landmark, id):
            """ Print the orientation of landmarks as a Euler Angle in degrees. """
            landmark = deepcopy(incoming_landmark)
            q = landmark.pose.orientation
            self.logger.debug("\n" + str(q), var_name = "quaternion" + str(id))
            self.logger.debug([round(degrees(t)) for t in tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])], var_name = "orientation" + str(id))

    LocalizationTest().run()