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
import matplotlib.pyplot as mpl

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
        self.num_mult = self.max_particles*self.percent_keep/10

        #ROS
        rospy.init_node('particle_manager')


    def initParticles(self, xy_yaw):
        """
        Place particles based on the robot's initial 2D estimated position
        """
        yaws = np.expand_dims(np.random.normal(xy_yaw[2], math.pi/6.0, self.max_particles), -1)
        xs = np.expand_dims(np.random.normal(xy_yaw[0], .75, self.max_particles), -1)
        ys = np.expand_dims(np.random.normal(xy_yaw[1], .75, self.max_particles), -1)
        weights = np.zeros_like(ys)
        self.current_particles = np.concatenate([xs, ys, yaws, weights],axis = 1)


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
        for particle in self.current_particles:

            yaws = np.expand_dims(np.random.normal(particle[2], math.pi/6.0, self.num_mult), -1)
            xs = np.expand_dims(np.random.normal(particle[0], .75, self.num_mult), -1)
            ys = np.expand_dims(np.random.normal(particle[1], .75, self.num_mult), -1)
            weights = np.zeros_like(ys)
            new_particles.append(np.concatenate([xs, ys, yaws, weights],axis = 1))
        new_particles = np.concatenate(new_particles, axis = 0)

        self.current_particles = np.concatenate([self.current_particles, new_particles], axis = 0)

    def deleteParticles(self, dt0_r_dt1, closest_point, OccupancyField):
        self.updateParticles(dt0_r_dt1)
        self.updateWeights(closest_point, OccupancyField)
        print(self.current_particles[:,3])
        indices_good = np.argsort(self.current_particles[:,3])
        self.current_particles = self.current_particles[indices_good[:int(self.max_particles * self.percent_keep)]]

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

class OF:
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




