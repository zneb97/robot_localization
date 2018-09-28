#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from localizer import SensorArray
from helper_functions import TFHelper
from occupancy_field import OccupancyField

normal_2d = [[.75,0],[0,.75]]

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return atan2(sin(z), cos(z))

class ParticleManager:

	def __init__(self):

		self.sensorManager = SensorArray()
		self.current_particles = []
		self.max_particles = 200
		self.percent_keep = 0.2
		self.num_mult = 4


		#ROS 
		rospy.init_node('particle_manager')

	def initParticles(self, xy_yaw):
		yaws = np.random.normal(xy_yaw[2], math.pi/6.0, self.max_particles)
		locations = np.random.multivariate_normal([xy_yaw[0], xy_yaw[1]], normal_2d, self.max_particles)

		for i in range(self.max_particles):
			new_particle = Particle(locations[i][0], locations[i][1], angle_normalize(yaws[i]))
			self.current_particles.append(new_particle)


	def getParticles(self):
		pass


	def addParticles(self):
		new_particles = []
		for particle in current_particles:

			yaws = np.random.normal(particle.yaw, math.pi/6.0, self.num_mult)
			locations = np.random.multivariate_normal([particle.x, particle.y], normal_2d, self.num_mult)

			for i in range(self.num_mult):
				new_particle = Particle(locations[i][0], locations[i][1], angle_normalize(yaws[i]))
				self.new_particles.append(new_particle)

		self.current_particles += new_particles

class Particle:

	def __init__(self, x, y, yaw):

		self.x = x
		self.y = y
		self.yaw = yaw
		self.weight = None
