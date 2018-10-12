#!/usr/bin/env python

"""
Ben Ziemann / Nick Steelman
Last updated: 10/11/18

Position, cull, and update particles as needed to determine location
of the robot
"""

from __future__ import print_function, division
import rospy
import numpy as np
import math 
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from sensorArray import SensorArray, angle_diff
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import matplotlib.pyplot as mpl


def angle_normalize(z):
    """
    Convenience function to map an angle to the range [-pi,pi]
    """
    return math.atan2(math.sin(z), math.cos(z))



class ParticleManager:
    def __init__(self):

        self.current_particles = []
        self.max_particles = 64 * 64 * 5
        self.percent_keep = 0.5
        self.num_mult = int(1/self.percent_keep) -1
        self.std_yaw =  math.pi/72.0
        self.std_pos = .01
        self.prop_yaw = 5 #proportion of variation in yaw
        # self.start_part = 1000 # number of particles to start the
        #ROS
        # rospy.init_node('particle_manager')


    def initParticles(self, xy_yaw):
        """
        Place particles based on the robot's initial 2D estimated position

        xy_yaw - Tuple of robots initial (x,y,yaw) map position
        """
        init_std = .3
        init_yaw = math.pi/2
        yaws = np.expand_dims(np.random.normal(xy_yaw[2], init_yaw, self.max_particles), -1)
        xs = np.expand_dims(np.random.normal(xy_yaw[0], init_std, self.max_particles), -1)
        ys = np.expand_dims(np.random.normal(xy_yaw[1], init_std, self.max_particles), -1)
        weights = np.zeros_like(ys)
        self.current_particles = np.concatenate([xs, ys, yaws, weights],axis = 1)


    def initParticlesHeading(self, xy_yaw, occupancy_field, closest_index):
        """
        Place particles based on the robot's alignment with the nearest obstacle

        xy_yaw - Tuple of robots initial (x,y,yaw) map position
        occupancy_filed - map of nearest occupencies for given coordinate
        closest_index - heading of the nearest obstacle based on robot laser scan
        """
        init_std = 1.5
        xs = np.expand_dims(np.random.normal(xy_yaw[0], init_std, self.max_particles), -1)
        ys = np.expand_dims(np.random.normal(xy_yaw[1], init_std, self.max_particles), -1)
        weights = np.zeros_like(ys)
        yaws = np.zeros_like(ys)

        #Calculate heading in relation to closest obstacle
        closest_index = math.radians(closest_index)
        for i in range(len(xs)):
            p_closest, obstacle_yaw = occupancy_field.get_closest_obstacle_distance(xs[i,0], ys[i,0])
            try:
                yaws[i,0] = angle_normalize(obstacle_yaw- closest_index)
            except:
                yaws[i,0] = 0.0

        self.current_particles = np.concatenate([xs, ys, yaws, weights],axis = 1)


    def getMapBounds(self, occupancy_field):
        """
        Get bounding box coordinates for the map
        """
        return occupancy_field.map.info.width, occupancy_field.map.info.height,\
                    0, 0


    def initParticlesUniform(self, occupancy_field):
        """
        This will initialize a set of points evenly spaced in the map with a
        variety of angles for each particle

        occupancy_filed - map of nearest occupencies for given coordinate
        """
        
        #Get map and spacing info
        of = occupancy_field
        m_width, m_height, x_off, y_off = self.getMapBounds(of)
        num_points = int(self.max_particles / self.prop_yaw)
        num_x = int(math.pow(num_points,.5))
        num_y = int(num_points // num_x)
        dist_x = int(m_width / num_x)
        dist_y = int(m_height / num_y)
        num_grid = num_x * num_y

        x_values = np.zeros((num_grid,1))
        for i in range(num_y):
            x_values[i*num_x: (i+1) * num_x,0] = np.full(num_x, i)
        y_values = np.expand_dims(np.tile(np.arange(num_y), [num_x]),-1)

        x_values = (x_values) * dist_x + dist_x//2
        y_values = (y_values) * dist_y + dist_y//2
        x_values = (x_values * of.map.info.resolution)+ of.map.info.origin.position.x
        y_values = (y_values * of.map.info.resolution)+ of.map.info.origin.position.y
        yaw_values = np.zeros((num_grid,1))
        weight_values = np.zeros((num_grid,1))
        points = np.concatenate([x_values, y_values, yaw_values, weight_values], axis = 1)
        points = np.tile(points, [self.prop_yaw,1])

        #Place particles
        inc = 2 * math.pi / self.prop_yaw
        for i in range(self.prop_yaw):
            beginRange = (i*num_points)
            endRange = ((i+1) * num_points)
            spread = np.full(num_points, i * inc)
            points[beginRange:endRange:1,2] = spread
        self.current_particles = points


    def getParticles(self):
        """
        Debugging functions, prints map location, heading, and weighting for
        each particle
        """

        for particle in current_particles:
            print("X: %f, Y: %f, Theta: %f, Weight: %f" %(particle[0], particle[1], particle[2], particle[3]))


    def addParticlesHeading(self, occupancy_field, closest_index):
        """
        First pass. Knowing we cull self.percent_keep of particles each iteration
        we add new ones around the remaining ones. Heading's based on nearest obstacle

        TODO: Distribute the number of particles around the current particles based on
        their weight

        occupancy_filed - map of nearest occupencies for given coordinate
        closest_index - heading of the nearest obstacle based on robot laser scan
        """
        new_particles = []
        for particle in self.current_particles:
            xs = np.expand_dims(np.random.normal(particle[0], self.std_pos, self.num_mult), -1)
            ys = np.expand_dims(np.random.normal(particle[1], self.std_pos, self.num_mult), -1)
            weights = np.zeros_like(ys)
            yaws = np.zeros_like(ys)
            closest_index = math.radians(closest_index)
            for i in range(len(xs)):
                p_closest, obstacle_yaw = occupancy_field.get_closest_obstacle_distance(xs[i,0], ys[i,0])
                yaws[i,0] = angle_normalize(obstacle_yaw + (math.pi/2) - closest_index)

            new_particles.append(np.concatenate([xs, ys, yaws, weights],axis = 1))

        new_particles = np.concatenate(new_particles, axis = 0)
        self.std_pos = max(0.1, self.std_pos - 0.002)
        self.current_particles = np.concatenate([self.current_particles, new_particles], axis = 0)


    def addParticles(self):
        """
        First pass. Knowing we cull 80% of particles each iteration
        we add 4 new ones around the remaining ones.

        TODO: Distribute the number of particles around the current particles based on
        their weight

        """
        self.current_particles[:,3] *= .5
        new_particles = []
        for particle in self.current_particles:
            yaws = np.expand_dims(np.random.normal(particle[2], self.std_yaw, self.num_mult), -1)
            xs = np.expand_dims(np.random.normal(particle[0], self.std_pos, self.num_mult), -1)
            ys = np.expand_dims(np.random.normal(particle[1], self.std_pos, self.num_mult), -1)
            weights = np.zeros_like(ys) + particle[3]
            new_particles.append(np.concatenate([xs, ys, yaws, weights],axis = 1))

        new_particles = np.concatenate(new_particles, axis = 0)
        # self.std_pos = max(0.01, self.std_pos - 0.002)
        # self.std_yaw = max(math.pi/36.0, self.std_yaw - math.pi/72.0)

        self.current_particles = np.concatenate([self.current_particles, new_particles], axis = 0)



    def deleteParticles(self, dt0_r_dt1, closest_point, closest_yaw, occupancy_field):
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
        self.updateWeights(closest_point, closest_yaw, occupancy_field)
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
        dt0 = dt0_r_dt1[0]#+np.random.normal(0, math.pi/72, len(self.current_particles))
        r = dt0_r_dt1[1]
        dt1 = dt0_r_dt1[2]
        yaw = yaw + dt0
        dx = r * np.cos(yaw)#if normal coordinate system
        dy = r * np.sin(yaw)
        yaw = yaw + dt1
        self.current_particles[:,2] = yaw
        self.current_particles[:,0] += dx
        self.current_particles[:,1] += dy


    def updateWeights(self, closest_point, closest_yaw, OccupancyField):
        """
        Update the weighting for a given particle based on distance to closest object
        compared to robot.
        closest_point - distance (meters) that the closest occupancy to the robot is
        occupancy_field - array of distance to closest occupancy for any given map coordinate
        """
        yaw_effect = 0.0
        for index in range(len(self.current_particles)):
            x = self.current_particles[index,0]
            y = self.current_particles[index,1]
            p_closest, obstacle_yaw = OccupancyField.get_closest_obstacle_distance(x, y)
            try:
                yaw_weight = abs(angle_diff(obstacle_yaw, closest_yaw)) #Between 0 and pi
            except:
                yaw_weight = np.inf
            dist_weight = abs(closest_point - p_closest)
            self.current_particles[index, 3] += yaw_weight*yaw_effect + dist_weight




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
