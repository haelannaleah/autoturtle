""" Create a safe wrapper for transformations.

Author:
    Annaleah Ernst
"""
from tf import TransformListener

class tfTransformer(TransformListener):

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
            return transform_func(self, target_frame, object)
        
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

    def transformQuaternion(self, target_frame, stamped_object):
        """ Transform stamped object from the frame in its header to the target frame. """
        return self._attemptLookup(TransformListener.transformQuaternion, target_frame, stamped_object)

    def transformPoint(self, target_frame, stamped_object):
        """ Transform stamped object from the frame in its header to the target frame. """
        return self._attemptLookup(TransformListener.transformPoint, target_frame, stamped_object)