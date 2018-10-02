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

#Covariance for multi-dimensional normalizaion
normal_2d = [[.75,0],[0,.75]]


def angle_normalize(z):
    """
    Convenience function to map an angle to the range [-pi,pi]
    """
    return atan2(sin(z), cos(z))


class ParticleManager:


	def __init__(self):

		self.sensorManager = SensorArray()
		self.current_particles = []
		self.max_particles = 200
		self.percent_keep = 0.2
		self.num_mult = max_particles*percent_keep/10

		#ROS
		rospy.init_node('particle_manager')


	def initParticles(self, xy_yaw):
		"""
		Place particles based on the robot's initial 2D estimated position
		"""
		yaws = np.expand_dims(np.random.normal(xy_yaw[2], math.pi/6.0, self.max_particles))
		xs = np.expand_dims(np.random.normal(xy_yaw[0], .75, self.max_particles))
		ys = np.expand_dims(np.random.normal(xy_yaw[1], .75, self.max_particles))
		weights = np.expand_dims(np.zeros_like(locations))
        self.current_particles = np.concatentate([xs, ys, yaws, weights],axis = 1)


	def getParticles(self):
		"""
		Debugging functions, prints map location, heading, and weighting for
		each particle
		"""

		for particle in current_particles:
			print("X: %f, Y: %f, Theta: %f, Weight: %f" %(particle[0], particle[1], particle[2], particle[3]))


    def addParticles(self):
        """
        First pass. Knowing we cull 80% of particles each iteration
        we add 4 new ones around the remaining ones

        TODO: Distribute the number of particles around the current particles based on
        their weight

        """
        new_particles = []
        for particle in current_particles:

			yaws = np.expand_dims(np.random.normal(paricle[2], math.pi/6.0, self.num_mult))
			xs = np.expand_dims(np.random.normal(paricle[0], .75, self.max_particles))
    		ys = np.expand_dims(np.random.normal(paricle[1], .75, self.max_particles))
		    weights = np.expand_dims(np.zeros_like(locations))
		    new_particles.append(np.concatentate([xs, ys, yaws, weights],axis = 1))
        new_particles = np.concatentate(new_particles, axis = 0)

        self.current_particles = np.concatentate([self.current_particles, new_particles], axis = 0)

    def deleteParticles(self, dt0_r_dt1, closest_point, OccupancyField):
        self.updateParticles(dt0_r_dt1)
        self.updateWeights(OccupancyField)
        indices_good = np.argmin(self.current_particles[:,3])
        self.current_particles = self.current_particles[indices_good]

    def updateParticles(self, dt0_r_dt1):
        yaw = self.current_particles[:,2]
        dt0, r, dt1 = dt0_r_dt1
        yaw = yaw + dt0
        dx = r * np.cos(yaw)#if normal coordinate system
        dy = r * np.sin(yaw)
        yaw = yaw + dt1
        self.current_particles[:,2] = yaw
        self.current_particles[:,0] += dx
        self.current_particles[:,1] += dy

    def updateWeights(self, closest_point, OccupancyField):
        for index in range(len(self.current_particles)):
            x = self.current_particles[index,0]
            y = self.current_particles[index,1]
            p_closest = OccupancyField.get_closest_obstacle_distance(x, y)
            self.current_particles[index, 3] = p_closest

if __name__ == "__main__":
    pm = ParticleManager
    pm.initParticles((0,0,0))
    print(pm.current_particles)
