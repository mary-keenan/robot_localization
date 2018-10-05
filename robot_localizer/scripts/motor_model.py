#!/usr/bin/env python

'''
CompRobo Particle Filter Motor Model ROS Node
@author: Connor Novak
@email: connorrnovak@gmail.com
@brief: Publishes pose diff to /m_model every time angle or dist traveled
exceeds thresholds
@version: 0.1
'''

# Import libraries
from __future__ import print_function, division         # Python 2 back-compatibility
import rospy as rp                                      # ROS python library
import numpy as np                                      # Used for pose diff math
from std_msgs.msg import Header                         # Used to define frame for Pose messages
from nav_msgs.msg import Odometry                       # /odom cb message type
from geometry_msgs.msg import Pose2D, Pose, PoseArray   # /m_model pub message type
from tf.transformations import euler_from_quaternion, quaternion_from_euler    # decoding odom heading info, recoding pose info
from helper_functions import TFHelper                   # Paul's provided helper functions


class MotorModel(object):
    # see @brief in .py script file header comment

    def __init__(self, tfhelper, visualize=True):

        # Variables
        self.dist_thresh = 0.2      # m
        self.angle_thresh = np.pi/4 # rad
        self.curr_pose = None
        self.prev_pose = None
        self.calc_pose = None
        self.visualize = visualize
        self.vis_array = PoseArray(header=Header(frame_id="/odom"))
        self.tfhelper = tfhelper

        # ROS constructs
        rp.init_node("motor_model")
        rp.Subscriber("/odom", Odometry, self.odom_cb)
        self.model_pub = rp.Publisher("/m_model", Pose2D, queue_size=2)
        self.update_rate = rp.Rate(2)

        if self.visualize:
            self.vis_pub = rp.Publisher("/m_model_poses", PoseArray, queue_size=1)

        self.wait_for_data()

        self.calc_pose = self.curr_pose

    def odom_cb(self, msg):
        # Callback function that updates current position
        self.curr_pose = msg.pose.pose


    def get_limits(self):
        # Returns distance and orientation difference thresholds
        return self.dist_thresh, self.angle_thresh


    def set_limits(self, d, theta):
        # Sets d and theta to distance and orientation difference thresholds
        self.dist_thresh = d
        self.angle_thresh = theta


    def wait_for_data(self):
        # Ensures sensors initialize

        while self.curr_pose == None and not rp.is_shutdown():
            rp.logwarn("No sensor data received on /odom")
            self.update_rate.sleep()

        rp.loginfo("Received sensor data on /odom")
        self.prev_pose = self.curr_pose
        return


    def update_vis_poses(self, pose2d):
        # Adds Pose to vis posearray that differs from previous Pose by x, y, theta
        xy_theta_prev = self.tfhelper.convert_pose_to_xy_and_theta(self.prev_pose)
        xy_theta_curr = xy_theta_prev + (pose2d.x, pose2d.y, pose2d.theta)

        translation = (xy_theta_curr[0], xy_theta_curr[1], 0)
        rotation = quaternion_from_euler(0,0,xy_theta_curr[2])
        self.vis_array.poses.append(self.tfhelper.convert_translation_rotation_to_pose(translation, rotation))
        self.vis_pub.publish(self.vis_array)


    def check_thresholds(self):
        # Returns boolean for whether or not to update output pose

        x, y, theta = self.get_pose_diff(self.prev_pose, self.curr_pose)
        if np.abs(x**2 + y**2) > self.dist_thresh:
            return True
        elif np.abs(theta) > self.angle_thresh:
            return True

        return False


    def get_pose_diff(self, p1, p2):
        # Returns dist diff and angle diff between two Poses
        x_diff = p2.position.x - p1.position.x
        y_diff = p2.position.y - p1.position.y
        theta_2 = euler_from_quaternion((p2.orientation.x,
                                         p2.orientation.y,
                                         p2.orientation.z,
                                         p2.orientation.w))
        theta_1 = euler_from_quaternion((p1.orientation.x,
                                         p1.orientation.y,
                                         p1.orientation.z,
                                         p1.orientation.w))
        theta_diff = theta_2[2] - theta_1[2]
        return x_diff, y_diff, theta_diff


    def run(self):
        # Runs main checking/publishing loop of motormodel

        while not rp.is_shutdown():

            # If pose or angle crosses thresh, send msg and update prev pose
            if self.check_thresholds():
                x, y, theta = self.get_pose_diff(self.prev_pose, self.curr_pose)
                pose = Pose2D(x=x, y=y, theta=theta)
                self.model_pub.publish(pose)
                self.prev_pose = self.curr_pose
                if self.visualize:
                    self.update_vis_poses(pose)

            self.update_rate.sleep()


if __name__ == "__main__":
    m_model = MotorModel(TFHelper())
    m_model.run()
