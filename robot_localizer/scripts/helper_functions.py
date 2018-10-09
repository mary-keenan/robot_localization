""" Some convenience functions for translating between various representations
    of a robot pose. """

from __future__ import print_function, division

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import tf.transformations as t
from tf import TransformListener
from tf import TransformBroadcaster

import math


class TFHelper(object):
    """ TFHelper Provides functionality to convert poses between various
        forms, compare angles in a suitable way, and publish needed
        transforms to ROS """
    def __init__(self):
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

    def convert_translation_rotation_to_pose(self, translation, rotation):
        """ Convert from representation of a pose as translation and rotation
            (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation[0],
                                   y=translation[1],
                                   z=translation[2]),
                    orientation=Quaternion(x=rotation[0],
                                           y=rotation[1],
                                           z=rotation[2],
                                           w=rotation[3]))

    def convert_pose_inverse_transform(self, pose):
        """ This is a helper method to invert a transform (this is built into
            the tf C++ classes, but ommitted from Python) """
        transform = t.concatenate_matrices(
            t.translation_matrix([pose.position.x,
                                  pose.position.y,
                                  pose.position.z]),
            t.quaternion_matrix([pose.orientation.x,
                                 pose.orientation.y,
                                 pose.orientation.z,
                                 pose.orientation.w]))
        inverse_transform_matrix = t.inverse_matrix(transform)
        return (t.translation_from_matrix(inverse_transform_matrix),
                t.quaternion_from_matrix(inverse_transform_matrix))

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = t.euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should
            be in radians) the difference is always based on the closest
            rotation from angle a to angle b.
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    def fix_map_to_odom_transform(self, robot_pose, timestamp):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer """
        (translation, rotation) = \
            self.convert_pose_inverse_transform(robot_pose)
        # print ("translation:", translation,"rotation:", rotation)

        p = PoseStamped(
            pose=self.convert_translation_rotation_to_pose(translation,
                                                           rotation),
            header=Header(stamp=timestamp, frame_id='base_link'))
        self.tf_listener.waitForTransform('base_link',
                                          'odom',
                                          timestamp,
                                          rospy.Duration(1.0))
        self.odom_to_map = self.tf_listener.transformPose('odom', p)
        (self.translation, self.rotation) = \
            self.convert_pose_inverse_transform(self.odom_to_map.pose)

    def send_last_map_to_odom_transform(self):
        if not(hasattr(self, 'translation') and hasattr(self, 'rotation')):
            return
        # print ("translation:",self.translation,"rotation:", self.rotation)
        self.tf_broadcaster.sendTransform(self.translation,
                                          self.rotation,
                                          rospy.get_rostime(),
                                          'odom',
                                          'map')
