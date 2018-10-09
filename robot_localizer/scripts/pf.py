#!/usr/bin/env python

'''
    CompRobo Particle Filter ROS Node
    @author: Connor Novak
    @email: connorrnovak@gmail.com
    @brief: Updates particle filter for every message from motor_model node
    @version: 0.0
'''

from __future__ import print_function, division
import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped, Pose, Pose2D, Vector3, Point
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from sensor_model import SensorModel
from robot_localizer.msg import WeightedPose, WeightedPoseArray
from tf.transformations import quaternion_from_euler
from tf import TransformListener


class ParticleFilter(object):
    # see @brief in .py script file header comment

    def __init__(self):
        rospy.init_node('pf')

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        rospy.Subscriber("/m_model", Pose2D, self.pf_loop)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)
        self.pose_estimate_pub = rospy.Publisher("poseestimate",
                                                 PoseStamped,
                                                 queue_size=10)

        # create instances of two helper objects
        rospy.loginfo("Initializing occupancy field")
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.transform_listener = TransformListener()
        rospy.loginfo("Initializing sensor model")
        self.sensor_model = SensorModel(self.occupancy_field, self.transform_helper)

        self.particles = WeightedPoseArray(header=Header())
        self.particle_num = 3
        self.loopcounter = 0


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """

        try:
            rospy.loginfo("Got new initial pose, resetting particle filter")
            xy_theta = \
                self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

            # TODO this should be deleted before posting
            self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
            self.transform_helper.send_last_map_to_odom_transform()

        except AttributeError:
            rospy.logwarn("Error: No static map provided")
            return

        # initialize your particle filter based on the xy_theta tuple
        particles = [None] * self.particle_num

        # creates initial random distribution of poses
        for particle in range(0,self.particle_num):
            pose = WeightedPose(weight=0.5, pose=PoseStamped(header=Header(frame_id='odom')))
            pose.pose.pose.position.x = np.random.normal(msg.pose.pose.position.x, 0.3)
            pose.pose.pose.position.y = np.random.normal(msg.pose.pose.position.y, 0.3)
            pose.pose.pose.position.z = 0
            pose.pose.pose.orientation = msg.pose.pose.orientation
            particles[particle] = self.transform_listener.transformPose('map', pose.pose)


        rospy.loginfo("Generated new batch of particles")
        self.particles.poses = particles
        self.update_particle_display()


    def update_particle_display(self):

        if len(self.particles.poses) > 0:
            particles = [p.pose for p in self.particles.poses]
            self.particle_pub.publish(header=Header(frame_id='map'), poses=particles)
        else:
            rospy.logwarn("Error: no particles to display")


    def propagate_particles(self, x, y, theta):

        trans_vals = (0, 0.05)   # mean center, std_dev
        rot_vals = (0, np.pi/12) # mean center, std_dev
        for particle in self.particles.poses:

            # Get current pose
            x_prev, y_prev, theta_prev = self.transform_helper.convert_pose_to_xy_and_theta(particle.pose)

            # Add noise to increments
            x_prev += np.random.normal(trans_vals[0], trans_vals[1])
            y_prev += np.random.normal(trans_vals[0], trans_vals[1])
            theta_prev += np.random.normal(rot_vals[0], rot_vals[1])

            # Apply increments
            translation = (x_prev + (np.sqrt(x**2+y**2)*np.cos(theta_prev)), y_prev + (np.sqrt(x**2+y**2)*np.sin(theta_prev)), 0)
            rotation = quaternion_from_euler(0,0, theta_prev + theta)
            particle.pose = self.transform_helper.convert_translation_rotation_to_pose(translation, rotation)

        self.update_particle_display()


    def update_particle_filter_position(self):
        # Returns average of all particles in particle filter as pose
        position = (0, 0)
        orientation_point = (0, 0)  # avg point 1 meter along each pose's main axis

        for particle in self.particles.poses:


            pose_info = self.transform_helper.convert_pose_to_xy_and_theta(particle.pose)
            position = (position[0] + pose_info[0], position[1] + pose_info[1])
            orientation_point  = (orientation_point[0] + np.cos(pose_info[2]), orientation_point[1] + np.sin(pose_info[2]))

        position = (position[0]/len(self.particles.poses), position[1]/len(self.particles.poses))
        orientation_point = (orientation_point[0]/len(self.particles.poses), orientation_point[1]/len(self.particles.poses))
        orientation = np.arctan2(orientation_point[1], orientation_point[0])
        print(orientation)
        pose = self.transform_helper.convert_translation_rotation_to_pose(translation=(position[0], position[1], 0),
                                                                          rotation = quaternion_from_euler(0, 0, orientation))

        #self.transform_helper.fix_map_to_odom_transform(pose, rospy.Time())
        self.pose_estimate_pub.publish(PoseStamped(header=Header(frame_id='map',stamp=rospy.Time()),pose=pose))
        return pose


    def pf_loop(self, msg):
        # Callback for motor model, runs single iteration of particle filter loop
        rospy.loginfo("Sensor loop " + str(self.loopcounter))

        # update weights based on sensor model
        # self.sensor_model.populate_error_poses(self.particles.poses)


        # TODO: take weighted avg of particles; best guess position

        # Updates laser scan visualization
        pointcloud = self.sensor_model.transform_scan_to_pose(self.particles.poses[0].pose, self.sensor_model.pc_scan)
        self.sensor_model.visualize_transformed_scan(pointcloud)

        # Resample particles based on weights


        # Propagate particles based on motor model
        self.propagate_particles(msg.x, msg.y, msg.theta)

        # Update particle filter best guess position
        self.update_particle_filter_position()

        self.loopcounter += 1


    def run(self):
        r = rospy.Rate(10)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
