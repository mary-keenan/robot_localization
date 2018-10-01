#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import numpy

from helper_functions import TFHelper
from occupancy_field import OccupancyField


class ParticleFilter(object):
	""" The class that represents a Particle Filter ROS Node
	"""
	def __init__(self):
		rospy.init_node('pf')
		r = rospy.Rate(5)

		rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose) # responds to selection of a new approximate robot location
		self.particle_publisher = rospy.Publisher("particlecloud", PoseArray, queue_size=10) # for rviz

		# initialaize helper objects 
		self.occupancy_field = OccupancyField()
		self.transform_helper = TFHelper()

		# initialize models
		self.motor_model = MotorModel()
		self.sensor_model = SensorModel()

		# initialize parameters
		self.num_particles = 10


		# initialize variables
		self.weighted_pose_array = []
		self.initialize_state_distribution()
		self.current_position = None


	def initialize_state_distribution(self):
		""" randomly populate self.weighted_pose_array """
		weight = 1.0 / self.num_particles # all points have the same weight to start
		for i in range(self.num_particles):
			location = 	numpy.random.randint(low = -10, high = 11, size = 2) # low is inclusive, high is exclusive
			theta = numpy.random.randint(low = 0, high = 360, size = 1)
			point = WeightedPose(location[0], location[1], theta, weight)
			rospy.loginfo(point)
			self.weighted_pose_array.append(point)


	def subscribe_to_motor_model(self):
		rospy.Subscriber("/robot_position", LaserScan, self.update_position)


	def update_position(self, position):
		self.current_position = position


	def subscribe_to_sensor_model(self):
		rospy.Subscriber("/robot_scanner", Pose2D, self.update_sensor_data)


	def update_sensor_data(self, sensor_data):
		self.sensor_data = sensor_data


	def update_particle_weights(self):
		""" based on sensor data """
		pass


	def resample_by_weights(self):

		pass


	def redistribute_points(self):
		""" based on movement + noise """


		pass


	def update_initial_pose(self, msg): 
		""" Callback function to handle re-initializing the particle filter
			based on a pose estimate. These pose estimates could be generated
			by another ROS Node or could come from the rviz GUI """
		xy_theta = \
			self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

		# TODO this should be deleted before posting
		self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
														msg.header.stamp)
		# initialize your particle filter based on the xy_theta tuple


	def run(self):
		""" """
		while self.movement is None:
			r.sleep()

		last_position = self.current_position

		while not(rospy.is_shutdown()):
			movement = self.motor_model.calculate_movement(last_position, self.current_position)
			
			# check if the movement was significant (all 0s if it wasn't) and skip this iteration if it wasn't
			if movement.x == 0 and movement.y == 0 and movement.theta == 0:
				continue

			last_position = self.current_position	



			# SENSOR READING

			# in the main loop all we do is continuously broadcast the latest
			# map to odom transform
			self.transform_helper.send_last_map_to_odom_transform()
			r.sleep()




class SensorModel(object):

	def __init__(object):
		rospy.init_node('sm')
		self.rate = rospy.Rate(100)

		rospy.Subscriber("/stable_scan", LaserScan, self.publish_data)
		self.sensor_publisher = rospy.Publisher("robot_scanner", LaserScan, queue_size = 10)



	def publish_data(self, data):
		""" publishes 2D pose of current position """


		


class MotorModel(object):

	def __init__(self):
		rospy.init_node('mm')
		self.rate = rospy.Rate(100)

		rospy.Subscriber("odom", Odometry, self.publish_position)
		self.position_publisher = rospy.Publisher("robot_position", Pose2D, queue_size = 10)

		# initialize variables
		self.current_pose = None

		# initialize parameters
		self.linear_threshold_for_change = .2
		self.angular_threshold_for_change = 15 # in degrees
		self.standard_deviation_of_dist = .25 #  what should the magnitude of the noise be?


	def publish_position(self, odom):
		""" publishes 2D pose of current position """

		# converts pose (geometry_msgs.Pose) to (x,y,yaw) tuple
		pose = odom.pose.pose
		orientation_tuple = (pose.orientation.x,
							 pose.orientation.y,
							 pose.orientation.z,
							 pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		current_pose = Pose2D(x = pose.position.x, y = pose.position.y, theta = angles[2])
		self.position_publisher.publish(current_pose)


	def calculate_movement(self, last_pose, current_pose):
		""" calculates the change in location and angle between two poses and returns it if it's significant"""
		
		# calculate linear and angular change from last position
		linear_x_change = current_pose.x - self.last_pose.x
		linear_y_change = current_pose.y - self.last_pose.y
		linear_total_change = math.sqrt(linear_x_change**2 + linear_y_change**2)
		angular_change = abs(current_pose.theta - self.last_pose.theta)

		# check if movement has surpassed threshold
		if linear_total_change >= self.linear_threshold_for_change or angular_change >= self.angular_threshold_for_change:
			movement = Pose2D(x = linear_x_change, y = linear_y_change, theta = angular_change)
			return movement

		return Pose2D(x = 0, y = 0, theta = 0)


	def add_noise(self, pose, movement):
		""" calculates linear and angular noise from distribution and returns noisy pose """
		center_of_dist = 0 # centered here because we want negative and positive values
		normal_dist_samples  = numpy.random.normal(center_of_dist, self.standard_deviation_of_dist, 2)

		# calculate noise that is proportional to the magnitude of the movement
		linear_x_noise = normal_dist_samples[0] * movement.x**2
		linear_y_noise = normal_dist_samples[0] * movement.y**2 
		angular_noise = normal_dist_samples[1] * movement.theta

		# update input pose based on noise
		updated_pose = Pose2D(x = pose.x + linear_x_noise, y = pose.y + linear_y_noise, theta = pose.theta + angular_noise)
		
		return updated_pose


	# def run(self):
	# 	while not rospy.is_shutdown():
	# 		self.rate.sleep()


class WeightedPose(object):

	def __init__(self, x, y, theta, weight):
		self.pose = Pose2D(x = x, y = y, theta = theta)
		self.weight = weight



if __name__ == '__main__':
	n = ParticleFilter()
	n.run()
