""" Compute the transformation between odometry and map.
    
Author:
    Annaleah Ernst
"""
import rospy
import tf
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from copy import deepcopy
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, QuaternionStamped, Point, Quaternion
from math import atan2, cos, sin, sqrt, pi
from time import time

from floorplan import FloorPlan
from logger import Logger
from tf_transformer import TfTransformer

class Localization(TfTransformer):
    """ Handle landmark detection and global localization.
    
    Args:
        point_ids (set): Unique identifier for each waypoint in the graph.
        locations (dict): Point_ids mapped to tuples representing locations.
        neighbors (dict): Point_ids mapped to lists containing other point_ids representing 
            the current node's neighbors.
        landmark_ids (set): Unique identifier for each landmark in the graph.
        landmark_positions (dict): Map AprilTag landmark ids to their absolute
            position on the floorplan.
        landmark_angles (dict): Map AprilTag landmark ids to their absolute
            position on the floorplan. This specifies the angle of rotation of the landmark in the 
            xy plane; ie, how much has its horizontal vector deviated from the x axis.
    
    Attributes:
        tags (geometry_msgs.msg.PoseStamped dict): A dict of all the AprilTags currently in view in 
            their raw form.
        tags_odom (geometry_msgs.msg.PoseStamped dict): Same as above, but in the odometry frame.
        floorplan (FloorPlan): The map of the current space as a floorplan.
    """
    _AR_FOV_LIMIT = 2.0 * pi / 15.0  # radians
    
    def __init__(self, point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles):
        
        # listen for frame transformations
        TfTransformer.__init__(self)
        
        # store raw tag data, data in the odom frame, and data in the base frame
        self.tags = {}
        self.tags_base = {}
        self.tags_odom = {}
        
        # set up the transformer between the map and ekf
        self._transform = self._transform = {"map_pos": Point(0,0,0), "map_angle": 0, "odom_pos": Point(0,0,0), "odom_angle": 0}
        self.floorplan = FloorPlan(point_ids, locations, neighbors, landmark_ids, landmark_positions, landmark_angles)
        
        # smooth data by selectively sampling
        self._prev_odom = [0,0,0,0,0,0,1]
        
        # set up logger and csv logging
        self._logger = Logger("Localization")
    
        # subscribe to raw tag data
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._tagCallback, queue_size=1)
    
    def _tagCallback(self, data):
        """ Extract and process tag data from the ar_pose_marker topic. """
        if data.markers:
            # use a list comprehension to convert the raw marker data into a dictionary of PoseStamped objects
            #   I promise, its less scary than it looks...
            self.tags = {marker.id : PoseStamped(marker.header, marker.pose.pose) for marker in data.markers}
            self.tags_odom = self._tfTransformTags('/odom')
            self._setTransform()
        
        else:
            # we don't see any tags, so empty things out
            self.tags = {}
            self.tags_odom = {}
    
    def _setTransform(self):
        """ Set the transformation between the odom frame and the map frame based on current tag info. """
    
        # attempt to get the id of the closest landmark
        try:
            t = self.tags
            
            # compute the closest (viable) tag by looking for the smallest distance squared from the robot base
            #   among tags that also appear in landmarks
            dist2, closest_id = min((t[id].pose.position.x**2 + t[id].pose.position.z**2, id)
                for id in t if (id in self.floorplan.landmarks and id in self.tags_odom))
        
        # the argument to min was an empty list; we don't see any familiar landmarks
        except (TypeError, ValueError) as e:
            return
        
        # make sure that the values don't change on us
        p = deepcopy(self.tags_odom[closest_id].pose.position)
        q = deepcopy(self.tags_odom[closest_id].pose.orientation)

        # check to make sure that we haven't just gotten a random noisy outlier.
        cur_odom = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if np.allclose(self._prev_odom, cur_odom, atol = 0.1, rtol = 0.05):
            self._transform["map_pos"] = self.floorplan.landmarks[closest_id].pose.position
            self._transform["map_angle"] = self.floorplan.landmarks[closest_id].angle
            self._transform["odom_pos"] = p
            self._transform["odom_angle"] = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1]
        else:
            self._logger.warn("Noisy tag signal. Ignoring.")

        self._prev_odom = cur_odom

    def _tfTransformTags(self, target_frame):
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
        
        # transform the visible tags that are in the viable field of view
        transformed = {}
        for id in self.tags:
        
            # make sure that the data coming in is in a viable frame of view, and ignore if it's not
            # experimentally, I found points more than 7pi/15 rad away from the x-axis gave junk data
            if abs(atan2(self.tags[id].pose.position.x, self.tags[id].pose.position.z)) > self._AR_FOV_LIMIT:
                self._logger.warn("Tag outside FOV. Ignoring.")
                continue
        
            # since the tag should always be roughly perpendicular to the ground, these values should be relatively small
            if np.isclose(self.tags[id].pose.orientation.x, 1, atol = 0.01) or np.isclose(self.tags[id].pose.orientation.y, 1, atol = 0.01):
                
                self._logger.warn("Tag outside acceptable orientation limits. Ignoring.")
                continue
            # if abs(self.tags[id].pose.orientation.x) > 0.75 or abs(self.tags[id].pose.orientation.y) > 0.75:
            #     continue

            # get the header from the current tag
            header = self.tags[id].header
            
            # set the time to show that we only care about the most recent available transform
            header.stamp = rospy.Time(0)
            
            # orientation data is in the ar_marker_<id> frame, so we need to update the starting frame
            #   (if we just transform from the optical frame, then turning the AR tag upside down affects the
            #   reported orientation)
            # this will get us the angle between the ARtag's x-axis and the robot base's x-axis
            header.frame_id = '/ar_marker_' + str(id)
            orientation = self._tf_listener.transformQuaternion(target_frame, QuaternionStamped(header, self.tags[id].pose.orientation))
            
            # make sure the look-up succeeded
            if orientation is None:
                continue
                
            # incoming position data is relative to the rgb camera frame, so we reset the header to the optical
            #   frame to get the correct position (note that this step is necessary since we're getting a shallow
            #   copy of the header)
            header.frame_id = '/camera_rgb_optical_frame'
            position = self._tf_listener.transformPoint(target_frame, PointStamped(header, self.tags[id].pose.position))
                         
            # make sure the look-up succeeded
            if position is None:
                continue
            
            # if we made it this far, then we can add our pose data to our dictionary!
            transformed[id] = PoseStamped(position.header, Pose(position.point, orientation.quaternion))
            
        return transformed
    
    def transformPoint(self, position, from_frame, to_frame):
        """ Compute coordinate transformation.
        
        Args:
            position (geometry_msgs.msg.Point): A position in the from_frame.
            from_frame (str, "map" or "odom") The frame that the incoming point is in.
            to_frame (str, "map" or "odom") The frame that the final point will be in.
        
        Returns:
            A geometry_msgs.msg.Point in the target frame.
        """
        from_pos = from_frame + "_pos"
        to_pos = to_frame + "_pos"
        
        # the amount we've moved since we logged a transform point
        dx = position.x - self._transform[from_pos].x
        dy = position.y - self._transform[from_pos].y
        
        # the amount we've rotated since we've logged a transform point
        delta = self._transform[to_frame + "_angle"] - self._transform[from_frame + "_angle"] #self._transform[from_frame + "_angle"] - self._transform[to_frame + "_angle"]
    
        # now, add this movement back in to last transform point in the desired frame
        x = self._transform[to_pos].x + dx * cos(delta) - dy * sin(delta)
        y = self._transform[to_pos].y + dx * sin(delta) + dy * cos(delta)
    
        return Point(x, y, 0)
    
    def transformAngle(self, angle, from_frame, to_frame):
        """ Transform an angle from the from frame to the to frame. """
        
        # compute transformation
        transformed_angle = self._transform[to_frame + "_angle"] + angle - self._transform[from_frame + "_angle"]
        
        # wrap angle, if necessary
        if transformed_angle > pi:
            transformed_angle -= self._TWO_PI
        elif transformed_angle < -pi:
            transformed_angle += self._TWO_PI
        
        return transformed_angle
    
    def _csvPose(self, landmark_pose):
        """ Convert pose object into csv data. """
        
        p = landmark_pose.position
        q = landmark_pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        return (["X", "Y", "Z", "qX", "qY", "qZ", "qW", "roll", "pitch", "yaw"],
                [p.x, p.y, p.z, q.x, q.y, q.z, q.w, roll, pitch, yaw])

    def _csvLogAR(self, test_name, tags, tag_type, folder):
        """ Log information on all tags currently in view. """
    
        tags = deepcopy(tags)
        for id in tags:
        
            # log each tag in a separate folder
            fields, data = self._csvPose(tags[id].pose)
            self._logger.csv(test_name + "_" + tag_type + "_marker" + str(id), fields, data, folder = folder)

    def csvLogRawTags(self, test_name, folder = "tests"):
        """ Log new raw tag data in separate files. """
        
        self._csvLogAR(test_name, self.tags, "raw", folder)

    def csvLogOdomTags(self, test_name, folder = "tests"):
        """ Log new position of AR tags relative to the robot base in separate files. """

        self._csvLogAR(test_name, self.tags_odom, "odom", folder)
    
    def csvLogTransform(self, test_name, folder = "tests"):
        """ Log the transformation from the ekf frame to the map frame. """
                            
        self._logger.csv(test_name + "_transform", ["X_map", "Y_map", "angle_map", "X_odom", "Y_odom", "angle_odom"],
                    [self._transform["map_pos"].x, self._transform["map_pos"].y, self._transform["map_angle"],
                            self._transform["odom_pos"].x, self._transform["odom_pos"].y, self._transform["odom_angle"]],
                    folder = folder)

    def shutdown(self):
        """ Shutdown localization. """
        
        self._logger.shutdown()

if __name__ == "__main__":
    import numpy as np
    from tester import Tester
    from math import degrees, pi
    from copy import deepcopy

    class LocalizationTest(Tester):
        """ Run localization tests. """
        def __init__(self):
            Tester.__init__(self, "Localization")
            
            # set up localization (including map)
            landmarks = {0, 1}
            landmark_positions = {0:(0,0), 1:(1,1)}
            landmark_orientations = {0:-pi/2, 1:pi/2}
            self.localization = Localization({},{},{},landmarks, landmark_positions, landmark_orientations)
            
            self.csvtestname = "estimation"

        def main(self):
            """ Run main tests. """
            self.localization.csvLogRawTags(self.csvtestname)
        
        def screenLog(self, landmark, id):
            """ Nicely parse landmarks into easily logable data. """
            self.logger.info("Frame: " + str(landmark.header.frame_id))
            self.logOrientation(landmark, id)
            self.logPosition(landmark, id)
            
        def logPosition(self, incoming_landmark, id):
            """ Print the position of landmarks in meters. """
            p = deepcopy(incoming_landmark.pose.position)
            self.logger.debug("\n" + str((p.x, p.y, p.z)), var_name = "position" + str(id))
        
        def logOrientation(self, incoming_landmark, id):
            """ Print the orientation of landmarks as a Euler Angle in degrees. """
            q = deepcopy(incoming_landmark.pose.orientation)
            self.logger.debug("\n" + str((q.x, q.y, q.z, q.w)), var_name = "quaternion" + str(id))
            self.logger.debug([round(degrees(t)) for t in tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])], var_name = "orientation" + str(id))

    LocalizationTest().run()