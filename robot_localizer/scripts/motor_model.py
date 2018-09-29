#!/usr/bin/env python

'''
CompRobo Particle Filter Motor Model ROS Node
@author: Connor Novak
@email: connorrnovak@gmail.com
@brief: Publishes pose diff to /m_model every time angle or dist traveled
exceeds thresholds
@version: 0.0
'''

# Import libraries
from __future__ import print_function, division         # Python 2 back-compatibility
import rospy as rp                                      # ROS python library
import numpy as np                                      # Used for pose diff math
from nav_msgs.msg import Odometry                       # /odom cb message type
from geometry_msgs.msg import Pose2D                    # /m_model pub message type
from tf.transformations import euler_from_quaternion    # decoding odom heading info


class MotorModel(object):
    # see @brief in .py script file header comment

    def __init__(self):

        # Variables
        self.dist_thresh = 0.5      # m
        self.angle_thresh = np.pi/2 # rad
        self.curr_pose = None
        self.prev_pose = None

        # ROS Classes
        rp.init_node("motor_model")
        self.odom_sub = rp.Subscriber("/odom", Odometry, self.odom_cb)
        self.model_pub = rp.Publisher("/m_model", Pose2D, queue_size=2)
        self.update_rate = rp.Rate(2)

        self.wait_for_data()


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


    def check_thresholds(self):
        # Returns boolean for whether or not to update output pose

        x, theta = self.get_pose_diff(self.prev_pose, self.curr_pose)
        if np.abs(x) > self.dist_thresh or np.abs(theta) > self.angle_thresh:
            return True
        return False


    def get_pose_diff(self, p1, p2):
        # Returns dist diff and angle diff between two Poses
        x_diff = p2.position.x - p1.position.x
        theta_2 = euler_from_quaternion((p2.orientation.x,
                                         p2.orientation.y,
                                         p2.orientation.z,
                                         p2.orientation.w))
        theta_1 = euler_from_quaternion((p1.orientation.x,
                                         p1.orientation.y,
                                         p1.orientation.z,
                                         p1.orientation.w))
        theta_diff = theta_2[2] - theta_1[2]
        return x_diff, theta_diff


    def run(self):
        # Runs main checking/publishing loop of motormodel

        while not rp.is_shutdown():

            # If pose or angle crosses thresh, send msg and update prev pose
            if self.check_thresholds():
                x, theta = self.get_pose_diff(self.prev_pose, self.curr_pose)
                self.model_pub.publish(Pose2D(x=x, theta=theta))
                self.prev_pose = self.curr_pose

            self.update_rate.sleep()


if __name__ == "__main__":
    m_model = MotorModel()
    m_model.run()
