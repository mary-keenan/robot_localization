#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose2D, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import numpy

from helper_functions import TFHelper
from occupancy_field import OccupancyField


# TODO -- should I try to do something that forces the arrows to stay in the map?

class ParticleFilter(object):
	""" The class that represents a Particle Filter ROS Node """

	def __init__(self):

		# initialize topic listener, publisher, and other ROS-specific stuff
		self.rate = rospy.Rate(2)
		rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose) # responds to selection of a new approximate robot location
		self.particle_publisher = rospy.Publisher("my_particlecloud", PoseArray, queue_size = 10) # for rviz
		self.pose_array_msg = PoseArray()
		self.pose_array_msg.header.frame_id = "map"
		self.pose_array_msg.header.stamp = rospy.Time.now()

		# initialaize helper objects 
		self.transform_helper = TFHelper()

		# initialize parameters
		self.num_particles = 10
		self.num_particles_to_resample_around = 3
		self.standard_deviation_of_dist = .5
		self.linear_threshold_for_change = .2
		self.angular_threshold_for_change = 10 # in degrees

		# initialize models
		self.motor_model = MotorModel(self.standard_deviation_of_dist, self.linear_threshold_for_change, self.angular_threshold_for_change)
		self.sensor_model = SensorModel()

		# initialize variables
		self.weighted_pose_array = None
		self.current_position = None


	def subscribe_to_motor_model(self):
		""" starts subcribing to processed position topic with a callback for saving the data"""
		rospy.Subscriber("robot_position", Pose2D, self.update_position)


	def update_position(self, position):
		""" stores the most recent processed position """
		self.current_position = position


	def resample_by_weights(self):
		""" picks top self.num_particles_to_resample_around particles and allocates the remaining particles around them based on their weights """

		# filter out Nones and sort weights by descending order
		filtered_weighted_pose_array = []

		for weighted_pose in self.weighted_pose_array:
			if not math.isnan(weighted_pose.weight):
				filtered_weighted_pose_array.append(weighted_pose)

		# if all of the weights are NaN, then keep our last list
		if len(filtered_weighted_pose_array) is 0:
			return

		filtered_weighted_pose_array.sort(key = lambda x: x.weight, reverse = True)

		# select top x particles to keep
		particles_to_keep = filtered_weighted_pose_array
		if len(filtered_weighted_pose_array) > self.num_particles_to_resample_around:
			particles_to_keep = filtered_weighted_pose_array[0:self.num_particles_to_resample_around]

		# clone the top particles based on their weight and populate weighted_pose_array
		sum_of_kept_weights = sum(particle.weight for particle in particles_to_keep)
		normalized_particles_to_keep = [WeightedPose(pose = particle.pose, weight = particle.weight / sum_of_kept_weights) for particle in particles_to_keep]
		self.weighted_pose_array = []

		for particle in normalized_particles_to_keep:
			# weights are by error, so a lower weight is actually better
			num_particles_to_clone = int(round(particle.weight * self.num_particles))
			particle_clone = WeightedPose(pose = particle.pose, weight = 1 / self.num_particles)
			for n in range(num_particles_to_clone):
				self.weighted_pose_array.append(particle_clone)


	def update_initial_pose(self, msg): 
		""" Callback function to handle re-initializing the particle filter
			based on a pose estimate. These pose estimates could be generated
			by another ROS Node or could come from the rviz GUI """

		self.weighted_pose_array = []
		x, y, theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
		self.transform_helper.fix_map_to_odom_transform(msg.pose.pose, msg.header.stamp)
		
		# initialize your particle filter based on the xy_theta tuple
		weight = 1.0 / self.num_particles # all points have the same weight to start
		for i in range(self.num_particles):
			particle_x = numpy.random.normal(loc = x, scale = self.standard_deviation_of_dist, size = 1) # low is inclusive, high is exclusive
			particle_y = numpy.random.normal(loc = y, scale = self.standard_deviation_of_dist, size = 1)
			particle_theta = numpy.random.normal(loc = theta, scale = self.standard_deviation_of_dist, size = 1)
			point = WeightedPose(Pose2D(x = particle_x, y = particle_y, theta = particle_theta), weight)
			self.weighted_pose_array.append(point)


	def update_pose_array_msg(self):
		""" converts weighted poses to PoseArray msg for publishing the particle cloud """
		pose_array = []

		for weighted_pose in self.weighted_pose_array:
			point = Point(x = weighted_pose.pose.x, y = weighted_pose.pose.y)
			quaternion = Quaternion(x = math.cos(weighted_pose.pose.theta), y = math.sin(weighted_pose.pose.theta))
			pose = Pose(position = point, orientation = quaternion)
			pose_array.append(pose)

		self.pose_array_msg.poses = pose_array


	def run(self):
		""" runs particle filter """

		self.subscribe_to_motor_model()

		# wait for first position data
		while (self.current_position is None or self.weighted_pose_array is None) and not rospy.is_shutdown():
			self.rate.sleep()

		# initialize with first position data
		last_position = self.current_position

		while not rospy.is_shutdown():

			# get robot motion
			movement = self.motor_model.calculate_movement(last_position, self.current_position)
			if movement.x == 0 and movement.y == 0 and movement.theta == 0: # verify movement was significant
				continue

			last_position = self.current_position # movement was significant, so update current position

			# update particle poses and weights
			self.weighted_pose_array = self.motor_model.update_particle_poses(self.weighted_pose_array, movement)
			self.weighted_pose_array = self.sensor_model.update_particle_weights(self.weighted_pose_array)

			# publish particle cloud
			self.update_pose_array_msg()
			self.particle_publisher.publish(self.pose_array_msg)

			# resample poses based on weights
			self.resample_by_weights()

			# in the main loop all we do is continuously broadcast the latest map to odom transform
			self.transform_helper.send_last_map_to_odom_transform()

			# pause before next iteration
			self.rate.sleep()


class SensorModel(object):

	def __init__(self):

		# initialize topic listenere
		rospy.Subscriber("/stable_scan", LaserScan, self.store_data)

		# initialaize helper objects 
		self.occupancy_field = OccupancyField()

		# initialize variables
		self.sensor_data = None


	def store_data(self, data):
		""" stores list of most current sensor data """
		self.sensor_data = data.ranges


	def update_particle_weights(self, weighted_poses):
		""" returns total distance error as the new weight for all particles in WeightedPose[] """
		weighted_pose_array = []
		for weighted_pose in weighted_poses:
			particle_error = self.get_specific_particle_distance_error(weighted_pose.pose)
			weighted_pose_array.append(WeightedPose(pose = weighted_pose.pose, weight = particle_error))

		return weighted_pose_array


	def get_specific_particle_distance_error(self, pose): # TODO make this more robust
		""" gets the total distance error for the sensor readings given some particle pose """

		if self.sensor_data is None:
			return 0

		# for the angles with distance data (0.0 means no reading), calculate the distance of the pose
		total_error = 0
		for degree in range(len(self.sensor_data)):
			distance = self.sensor_data[degree]
			if distance is not 0.0:
				total_error += self.calculate_distance_error(pose, degree, distance)

		return total_error


	def calculate_distance_error(self, pose, sensor_degree, sensor_dist):
		""" calculates the distance from the closest object of the x, y position of the 
		point sensor_degree angle, sensor_dist away from the pose """
		
		# calculate the new x and y coordinates
		angle_in_radians = math.radians(pose.theta + sensor_degree)
		new_x = pose.x + math.cos(angle_in_radians) * sensor_dist
		new_y = pose.y + math.sin(angle_in_radians) * sensor_dist

		# use the helper object to find how far the new x, y positions are from an object
		# if this particle's pose is correct, the distance should be 0
		dist_error = self.occupancy_field.get_closest_obstacle_distance(new_x, new_y)
		return dist_error


class MotorModel(object):

	def __init__(self, standard_deviation_of_dist = .25, linear_threshold_for_change = .01, angular_threshold_for_change = 1):

		# initialize topic listeners and publisher
		rospy.Subscriber("odom", Odometry, self.publish_position)
		self.position_publisher = rospy.Publisher("robot_position", Pose2D, queue_size = 10)

		# initialize parameters
		self.linear_threshold_for_change = linear_threshold_for_change
		self.angular_threshold_for_change = angular_threshold_for_change
		self.standard_deviation_of_dist = standard_deviation_of_dist # determines magnitude of noise


	def publish_position(self, odom):
		""" publishes 2D pose of current position """

		# converts pose (geometry_msgs.Pose) to (x,y,yaw) tuple
		pose = odom.pose.pose
		orientation_tuple = (pose.orientation.x,
							 pose.orientation.y,
							 pose.orientation.z,
							 pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		current_pose = Pose2D(x = pose.position.x, y = pose.position.y, theta = math.degrees(angles[2]))
		self.position_publisher.publish(current_pose)


	def calculate_movement(self, last_pose, current_pose):
		""" calculates the change in location and angle between two poses and returns it if it's significant"""
		
		# calculate linear and angular change from last position
		linear_x_change = current_pose.x - last_pose.x
		linear_y_change = current_pose.y - last_pose.y
		linear_total_change = math.sqrt(linear_x_change**2 + linear_y_change**2)
		angular_change = abs(current_pose.theta - last_pose.theta)

		# check if movement has surpassed threshold
		if linear_total_change >= self.linear_threshold_for_change or angular_change >= self.angular_threshold_for_change:
			movement = Pose2D(x = linear_x_change, y = linear_y_change, theta = angular_change)
			return movement

		return Pose2D(x = 0, y = 0, theta = 0)


	def update_particle_poses(self, weighted_pose_array, movement):
		""" updates particle positions given movement, adds random noise """
		updated_weighted_pose_array = []

		for weighted_pose in weighted_pose_array:
			updated_pose = self.add_noisy_movement_to_particle(weighted_pose.pose, movement)
			updated_weighted_pose_array.append(WeightedPose(pose = updated_pose, weight = weighted_pose.weight))

		return updated_weighted_pose_array


	def add_noisy_movement_to_particle(self, pose, movement):
		""" calculates linear and angular noise from distribution given some movement and returns pose updated with noisy movement """
		center_of_dist = 0 # centered here because we want negative and positive values
		position_normal_dist_samples  = numpy.random.normal(loc = center_of_dist, scale = self.standard_deviation_of_dist, size = 3)
		theta_normal_dist_samples  = numpy.random.normal(loc = center_of_dist, scale = self.standard_deviation_of_dist/10, size = 3)

		# calculate noise that is proportional to the magnitude of the movement
		linear_x_noise = position_normal_dist_samples[0] * movement.x
		linear_y_noise = position_normal_dist_samples[1] * movement.y
		angular_noise = theta_normal_dist_samples[2] * movement.theta

		# print ("linear: %d and angular: %d" % (movement.x, movement.theta))

		# update input pose based on movement and noise
		updated_pose = Pose2D(x = pose.x + linear_x_noise, y = pose.y + linear_y_noise, theta = pose.theta + angular_noise)
		
		return updated_pose


class WeightedPose(object):

	def __init__(self, pose, weight):
		self.pose = pose
		self.weight = weight



if __name__ == '__main__':
	rospy.init_node('pf')
	n = ParticleFilter()
	n.run()
