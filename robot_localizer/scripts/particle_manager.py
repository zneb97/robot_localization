#!/usr/bin/env python

"""
Ben Ziemann / Nick Steelman
Last updated: 10/3/18

Position, cull, and update particles as needed to determine location
of the robot
"""

from __future__ import print_function, division
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from sensorArray import SensorArray
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import matplotlib.pyplot as mpl

#Covariance for multi-dimensional normalizaion


def angle_normalize(z):
    """
    Convenience function to map an angle to the range [-pi,pi]
    """
    return atan2(sin(z), cos(z))


class ParticleManager:
    def __init__(self):

        self.current_particles = []
        self.max_particles = 200
        self.percent_keep = 0.2
        self.num_mult = self.max_particles*self.percent_keep/10
        self.std_yaw =  math.pi/12.0
        self.std_pos = .1
        self.prop_yaw = 5 #proportion of variation in yaw
        # self.start_part = 1000 # number of particles to start the
        #ROS
        # rospy.init_node('particle_manager')


    def initParticles(self, xy_yaw):
        """
        Place particles based on the robot's initial 2D estimated position

        xy_yaw - Tuple of robots initial (x,y,yaw) map position
        """
        init_std = .05
        yaws = np.expand_dims(np.random.normal(xy_yaw[2], self.std_yaw, self.max_particles), -1)
        xs = np.expand_dims(np.random.normal(xy_yaw[0], init_std, self.max_particles), -1)
        ys = np.expand_dims(np.random.normal(xy_yaw[1], init_std, self.max_particles), -1)
        weights = np.zeros_like(ys)
        self.current_particles = np.concatenate([xs, ys, yaws, weights],axis = 1)

    def initParticlesUniform(self, occupancy_field):
        """ This will initialize a set of points evenly spaced in the map with a
        variety of angles for each particle """
        of = occupancy_field
        num_points = self.max_particles / self.prop_yaw
        num_x = int(math.pow(num_points,.5))
        num_y = num_points // num_x
        dist_x = int(of.map.info.width / num_x)
        dist_y = int(of.map.info.height / num_y)
        num_grid = num_x * num_y

        x_values = np.zeros((num_grid,1))
        for i in range(num_y):
            x_values[i*num_x: (i+1) * num_x,1] = np.full(num_x, i)
        y_values = np.expand_dims(np.tile(np.arange(num_y), [num_x]),-1)

        x_values = x_values * dist_x + dist_x//2
        y_values = y_values * dist_y + dist_y//2
        x_values = (x_values * of.map.info.resolution)+ of.map.info.origin.position.x
        y_values = (y_values * of.map.info.resolution)+ of.map.info.origin.position.y
        yaw_values = np.zeros((num_grid,1))
        weight_values = np.zeros((num_grid,1))
        points = np.concatenate([x_values, y_values, yaw_values, weight_values], axis = 1)
        points = np.tile(points, [self.prop_yaw,1])

        inc = 2 * math.pi / self.prop_yaw
        for i in range(self.prop_yaw):
            points[i*num_points: (i+1) * num_points,2] = np.full(points, i * inc)
        self.current_particles = points


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
        we add 4 new ones around the remaining ones.

        TODO: Distribute the number of particles around the current particles based on
        their weight

        """
        new_particles = []
        for particle in self.current_particles:
            yaws = np.expand_dims(np.random.normal(particle[2], self.std_yaw, self.num_mult), -1)
            xs = np.expand_dims(np.random.normal(particle[0], self.std_pos, self.num_mult), -1)
            ys = np.expand_dims(np.random.normal(particle[1], self.std_pos, self.num_mult), -1)
            weights = np.zeros_like(ys)
            new_particles.append(np.concatenate([xs, ys, yaws, weights],axis = 1))
        new_particles = np.concatenate(new_particles, axis = 0)

        self.current_particles = np.concatenate([self.current_particles, new_particles], axis = 0)


    def deleteParticles(self, dt0_r_dt1, closest_point, occupancy_field):
        """
        Cull particles based on distance to nearest object

        dt0_r_dt1 - tuple for the robots movement to map onto particles
            dt0 - how much the robot turned from its original location to drive in a straight line to new location
            r - distance (meters) how far to drive to new position
            dt1 - change from dt0 to robot's current heading
        closest_point - distance (meters) that the closest occupancy to the robot is
        occupancy_field - array of distance to closest occupancy for any given map coordinate
        """

        self.updateParticles(dt0_r_dt1)
        self.updateWeights(closest_point, occupancy_field)
        indices_good = np.argsort(self.current_particles[:,3])
        self.current_particles = self.current_particles[indices_good[:int(self.max_particles * self.percent_keep)]]


    def updateParticles(self, dt0_r_dt1):
        """
        Update particles' position based on robot's movements

        dt0_r_dt1 - tuple for the robots movement to map onto particles
            dt0 - how much the robot turned from its original location to drive in a straight line to new location
            r - distance (meters) how far to drive to new position
            dt1 - change from dt0 to robot's current heading
        """

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
        """
        Update the weighting for a given particle based on distance to closest object
        compared to robot.
        closest_point - distance (meters) that the closest occupancy to the robot is
        occupancy_field - array of distance to closest occupancy for any given map coordinate
        """

        for index in range(len(self.current_particles)):
            x = self.current_particles[index,0]
            y = self.current_particles[index,1]
            p_closest = OccupancyField.get_closest_obstacle_distance(x, y)

            self.current_particles[index, 3] = abs(closest_point - p_closest)




class OF:
    """
    Debugging class to simulate simple occupancy field when running particle_manager solo
    """
    def __init__(self, point):
        self.x, self.y = point[0], point[1]

    def get_closest_obstacle_distance(self,x,y):
        return np.linalg.norm([self.x - x, self.y - y])


if __name__ == "__main__":
    pm = ParticleManager()
    pm.initParticles((0,0,0))
    print(pm.current_particles)
    mpl.figure()
    mpl.scatter(pm.current_particles[:,0], pm.current_particles[:,1], c='b')
    of = OF((3,0))
    pm.deleteParticles((math.pi/2,2,0), .5, of)
    mpl.scatter(pm.current_particles[:,0], pm.current_particles[:,1], c='r')
    mpl.figure()
    x,y = pm.current_particles[:,0], pm.current_particles[:,1]
    pm.addParticles()
    mpl.scatter(pm.current_particles[:,0], pm.current_particles[:,1], c='g')
    mpl.scatter(x,y, c='r')
    mpl.show()
