#!/usr/bin/env python

'''
CompRobo Particle Filter Sensor Model ROS Node
@author: Connor Novak
@email: connorrnovak@gmail.com
@brief: Class for calculating sensor info for particles
in particle filter based on map
@version: 0.0
'''

# Import libraries
from __future__ import print_function, division
import rospy as rp
import numpy as np
import tf
from occupancy_field import OccupancyField
from robot_localizer.msg import WeightedPose, WeightedPoseArray
from geometry_msgs.msg import Pose, PoseArray, Point, PointStamped, TransformStamped, Vector3
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Header

class SensorModel():
    # see @brief in .py script file header comment

    def __init__(self, o_field, tf_helper):

        # Variables
        self.o_field = o_field
        self.tf_helper = tf_helper
        self.listener = tf.TransformListener()
        rp.Subscriber('/stable_scan', LaserScan, self.laser_cb)
        self.scan_pub = rp.Publisher('/visualize_scan', PointCloud, queue_size=1)
        self.pc_scan = None
        self.particle_tf_frame = TransformStamped(header=Header(frame_id='map', stamp=rp.Time(0)), child_frame_id='particle')


    def laser_cb(self, msg):
        # Laser scan callback, stores laser points
        self.pc_scan = msg


    def populate_error_poses(self, w_poses):
        # Updates weighted pose array based on average dist from pose scan data
        # to map

        errors = [None] * len(w_poses.poses)

        # calculate error for each pose
        for i in range(0, len(w_poses.poses)):
            errors[i] = (self.get_average_distance(w_poses.poses[i]))**2

        # normalize errors between 0 and 1, with large errors given low values
        max_error = max(errors)

        for i in range(0, len(errors)):
            if max_error == 0:
                errors[i] = 1
            else:
                errors[i] = 1 - errors[i]/max_error

        # Add normalized errors to poses as weights
        for i in range(0,len(w_poses.poses)):
            w_poses.poses[i].weight = errors[i]

        return w_poses


    def get_average_distance(self, pose):
        # Returns error based on avg dist to obstacle of each scan pt as taken from pose

        total_distance = 0

        # Translate scan to pose w/ respect to map
        (translation,rotation) = self.listener.lookupTransform('/base_link', '/map', rp.Time(0))
        map_pose = self.tf_helper.convert_translation_rotation_to_pose(translation, rotation)

        map_pose = self.transform_scan_to_pose(map_pose, self.pc_scan)
        # Check scan data avg dist to obstacle using OccupancyField
        for point in map_pose.points:
            total_distance += self.o_field.get_closest_obstacle_distance(point.x, point.y)

        # Return average distance
        return total_distance/len(self.pc_scan.ranges)


    def transform_scan_to_pose(self, pose, scan):
        # Transforms scan as if taken from pose

        # Catches no data case
        if scan == None:
            rp.logwarn("ERR: no scan data received - particle filter unable to verify sensor model")
            return


        pose_info = self.tf_helper.convert_pose_to_xy_and_theta(pose)
        point_list = [None] * len(scan.ranges)

        for i in range(0, len(scan.ranges)):
            dist = scan.ranges[i]
            angle = scan.angle_min + i * scan.angle_increment

            point_cartesian = PointStamped(Header(frame_id="base_laser_link"), Point(x=dist*np.cos(angle), y=dist*np.sin(angle), z=0))
            point_tf_base = self.listener.transformPoint("base_link", point_cartesian)
            point_tf_particle = Point(x=point_tf_base.point.x * np.cos(pose_info[2]) - point_tf_base.point.y * np.sin(pose_info[2]) + pose_info[0],
                                      y=point_tf_base.point.x * np.sin(pose_info[2]) + point_tf_base.point.y * np.cos(pose_info[2]) + pose_info[1])
            point_list[i] = point_tf_particle

        return PointCloud(header=Header(frame_id='map'), points=point_list)


    def visualize_transformed_scan(self, pc):
        # Show laserscan points in map as taken from pose

        if pc == None:
            rp.logwarn("Err: no points to plot")
            return

        # Publish points in map
        self.scan_pub.publish(pc)
