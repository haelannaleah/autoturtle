""" Global localization.
    
Author:
    Annaleah Ernst
"""
import rospy
import tf
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, QuaternionStamped, Point, Quaternion
from math import atan2, cos, sin, sqrt, pi

from logger import Logger

class Localization():
    """ Handle landmark detection and global localization.
    
    Args:
        landmarks (FloorPlan.Landmarks dict): A dict mapping the map position of landmarks to
            their AR tag ids.
    
    Attributes:
        tags (geometry_msgs.msg.PoseStamped dict): A dict of all the AprilTags currently in view in 
            their raw form.
        tags_base (geometry_msgs.msg.PoseStamped dict): Same as above, but in the robot base frame.
        tags_odom (geometry_msgs.msg.PoseStamped dict): Same as above, but in the odom frame.
        self.estimated_pose (geometry_msgs.msg.Pose or None): The estimated pose of the robot based 
            on the visible tags. None if no tags visible.
    """
    def __init__(self, landmarks):
        self._logger = Logger("Localization")
        
        # store raw tag data, data in the odom frame, and data in the base frame
        self.tags = {}
        self.tags_base = {}
        self.tags_odom = {}
        
        # set estimated pose based on local landmarks to None and set up the landmark map
        self.estimated_pose = None
        self.landmarks = landmarks
    
        # listen for frame transformations
        self._tf_listener = tf.TransformListener()
    
        # subscribe to raw tag data
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._tagCallback, queue_size=1)
    
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
        
    def _estimatePose(self):
        """ Estimate current position based on proximity to landmarks. """
        
        # attempt to get the id of the closest landmark
        try:
            t = self.tags_base
            
            # compute the closest (viable) tag by looking for the smallest distance squared from the robot base
            #   among tags that also appear in landmarks
            dist2, closest_id = min((t[id].pose.position.x**2 + t[id].pose.position.y**2, id) for id in t if id in self.landmarks)
        
        # the argument to min was an empty list; we don't see any familiar landmarks
        except (TypeError, ValueError) as e:
            self.estimated_pose = None
            return
        
        # extract the closest tag and corresponding landmark
        closest = self.tags_base[closest_id].pose
        map = self.landmarks[closest_id]
        
        # compute the value of the radius between the robot base and the ARtag
        r = sqrt(dist2)
        
        # Note: in the following section, names of angles correspond to symbols in graph
        #   <TODO include graph number>
        # get the angle between the ARtag's x-axis and the map's x-axis
        alpha = map.angle
        
        # get the angle between the ARtag's x-axis and the robot's x-axis (between 0 and -pi)
        q = closest.orientation
        beta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1]
        
        # get the angle between the robot's x-axis and the vector from the robot base to the tag
        gamma = atan2(closest.position.y, closest.position.x)

        # compute the angle between the x-axis in the robot frame and the x-axis in the map frame
        delta = alpha - beta

        # now compute the angle between the map x-axis and the vector to the AR tag
        theta = delta + gamma
        
        # compute the robot position in the map frame
        x = map.pose.position.x - r * cos(theta)
        y = map.pose.position.y - r * sin(theta)
        
        # plug this into an estimated pose
        q = tf.transformations.quaternion_from_euler(0,0,delta)
        self.estimated_pose = Pose(Point(x,y,0), Quaternion(q[0], q[1], q[2], q[3]))
        
        # log everything to verify that it makes sense
        self._logger.debug("POSE AND ANGLE")
        self._logger.debug(closest_id)
        self._logger.debug(self.estimated_pose)
        self._logger.debug(alpha, var_name = "alpha")
        self._logger.debug(beta, var_name = "beta")
        self._logger.debug(gamma, var_name = "gamma")
        self._logger.debug(delta, var_name = "delta")
        self._logger.debug(theta, var_name = "theta")
        self._logger.debug(r, var_name = "radius")
        
    def _tagCallback(self, data):
        """ Extract and process tag data from the ar_pose_marker topic. """
        if data.markers:
            # use a list comprehension to convert the raw marker data into a dictionary of PoseStamped objects
            #   I promise, its less scary than it looks...
            self.tags = {marker.id : PoseStamped(marker.header, marker.pose.pose) for marker in data.markers}
            self.tags_base = self._transformTags('/base_footprint')
            self.tags_odom = self._transformTags('/odom')
            self._estimatePose()
        else:
            # we don't see any tags, so empty things out
            self.tags = {}
            self.tags_base = {}
            self.tags_odom = {}

    def _transformTags(self, target_frame):
        """ Convert all of the visible tags to target frame.
        
        Args:
            target_frame (string): The desired final coordinate frame.
            
        Returns:
            A geometry_msgs.msg.PoseStamped dictionary containing the positions in the target frame
                of the visible AprilTags that were successfully transformed.
                
        Note: 
            Raw tag orientation data comes in the /ar_marker_<id> frame, and its position data comes in the
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
            # this will get us the angle between the ARtag's x-axis and the robot base's x-axis
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
    from math import degrees, pi
    from copy import deepcopy
    from floorplan import FloorPlan

    class LocalizationTest(Tester):
        """ Run localization tests. """
        def __init__(self):
            Tester.__init__(self, "Localization")
            
            # set up localization (including map)
            landmarks = {0, 1}
            landmark_positions = {0:(0,0), 1:(1,1)}
            landmark_orientations = {0:-pi/2, 1:pi/2}
            self.floorplan = FloorPlan({},{},{},landmarks, landmark_positions, landmark_orientations)
            self.localization = Localization(self.floorplan.landmarks)
            
            self.prev = {}
    
            fields = ["px", "py", "pz", "qx", "qy", "qz", "qw", "r", "p", "y"]
            
            self.csvfields = []
            self.csvfields.extend(["raw_" + field for field in fields])
            self.csvfields.extend(["odom_" + field for field in fields])
            self.csvfields.extend(["base_" + field for field in fields])
            self.csvtestname = "stationary"

        def main(self):
            """ Run main tests. """
            pass
        
        def logData(self):
            """ Log CSV file and output data to screen. """
            
            # make sure the tags don't go changing on us
            tags = deepcopy(self.localization.tags)
            landmarks_odom = deepcopy(self.localization.tags_odom)
            landmarks_relative = deepcopy(self.localization.tags_base)
            
            # separately log all tag data
            for id in tags:
            
                # make sure that the landmark data is in
                if id in landmarks_odom and id in landmarks_relative:
                
                    # convert landmark into csv data
                    csvdata = []
                    csvdata.extend(self.csvPose(tags[id]))
                    csvdata.extend(self.csvPose(landmarks_odom[id]))
                    csvdata.extend(self.csvPose(landmarks_relative[id]))

                    # if we've never encountered this marker before, or it's values have changed
                    if id not in self.prev or not np.allclose(csvdata, self.prev[id], rtol=.1, atol=.5):
                        self.prev[id] = csvdata
                        test_name = self.csvtestname + "_marker" + str(id)
                        
                        # if we've never encountered this marker before, open a new csv file
                        if not self.logger.isLogging(test_name):
                            self.logger.csv(test_name, self.csvfields, folder = "tests")
                    
                        # write data to the csv file
                        self.logger.csv(test_name, csvdata, folder = "tests")

                        # log data to the screen as well
                        self.screenLog(tags[id], id)
                        self.screenLog(landmarks_odom[id], id)
                        self.screenLog(landmarks_relative[id], id)
        
        def screenLog(self, landmark, id):
            """ Nicely parse landmarks into easily logable data. """
            self.logger.info("Frame: " + str(landmark.header.frame_id))
            self.logOrientation(landmark, id)
            self.logPosition(landmark, id)
    
        def csvPose(self, landmark):
            """ Convert pose object into csv data. """
            p = landmark.pose.position
            q = landmark.pose.orientation
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            return [p.x, p.y, p.z, q.x, q.y, q.z, q.w, roll, pitch, yaw]
            
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