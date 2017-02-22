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
            #   frame to get the correct position
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
        
            self.prev_relative = {}
            self.prev_odom = {}

        def main(self):
            """ Run main tests. """
            self.slowLogging(self.prev_relative, self.localization.landmarks_relative)
            self.slowLogging(self.prev_odom, self.localization.landmarks_odom)
    
        def slowLogging(self, prevs, landmarks):
            """ Only log things on updates! """
            for id in landmarks:
                if id not in prevs not self.similar(prevs[id], landmarks[id]):
                    self.logger.info("Frame: " + str(landmarks[id].header.frame_id))
                    self.logOrientation(landmarks[id])
                    self.logPosition(landmarks[id])
                    prevs[id] = landmarks[id].pose
    
        def similar(self, prev, landmark):
            """ Check to see if there have been significant changes in positions. """
            # store these in shorted named variables for notational reasons
            p_cur = landmark.pose.position
            q_cur = landmark.pose.orientation
        
            # check that the positions are close
            close_position = np.isclose([p_cur.x, p_cur.y, p_cur.z], [prev.position.x, prev.position.y, prev.position.z], atol=.05).all()
            
            # check that the orientation is close
            close_orient = np.isclose([q_cur.x, q_cur.y, q_cur.z, q_cur.w], [prev.orientation.x, prev.orientation.y, prev.orientation.z, prev.orientation.w], atol=.05).all()
        
            return close_orient and close_position
            
            
        def logPosition(self, incoming_landmark):
            """ Print the position of landmarks in meters. """
            landmark = deepcopy(incoming_landmark)
            self.logger.debug("\n" + str(landmark.pose.position), var_name = id)
        
        def logOrientation(self, incoming_landmark, id):
            """ Print the orientation of landmarks as a Euler Angle in degrees. """
            landmark = deepcopy(incoming_landmark)
            q = landmark.pose.orientation
            self.logger.debug([round(degrees(t)) for t in tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])], var_name = id)

    LocalizationTest().run()