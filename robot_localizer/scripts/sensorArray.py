#!/usr/bin/env python
"""
Ben Ziemann / Nick Steelman
Last updated: 10/11/18

Handle the neato's sensors intake
"""

import rospy
import math
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from math import sin, cos, atan2, pi, fabs


def distance(x,y):
    """
    Return euclidean distance from (0,0) to input (x,y)
    """
    return np.linalg.norm(np.array(x)-np.array(y))


def angle_normalize(z):
    """
    Convenience function to map an angle to the range [-pi,pi]
    """
    return atan2(sin(z), cos(z))


def angle_diff(a, b):
    """
    Finds the distance between any two angles.
    Out put between [0, pi]
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2


class SensorArray:

    def __init__(self):

        self.debug_on = True

        #Robot properities
        self.old_x = None
        self.old_y = None
        self.old_yaw = None
        self.x = None
        self.y = None
        self.yaw = None

        #Trigger this to record a laser scan
        self.laser_flag = False

        #Distance to closest object to the robot
        self.closest_dist = None
        self.min_index = None

        #ROS
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)
        rospy.Subscriber("/odom", Odometry, self.setLocation)


    def setLocation(self, msg):
        """
        Convert pose (geometry_msgs.Pose) to a (x, y, yaw) tuple
        Constantly being called as it is the callback function for this node's subscription

        odom is Neato ROS' nav_msgs/Odom msg composed of pose and orientation submessages
        """
        pose = msg.pose.pose
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)

        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = angles[2]

        if self.old_x is None:
            self.setOld()


    def getDelta(self):
        """
        Find the change in the robot's heading and position in terms of yaws
        and radius. these can then be passed to our particles to find their new
        positions
        """
        r = distance((self.old_x,self.old_y),(self.x,self.y)) #Linear distance traveled
        t_yaw = atan2(self.y - self.old_y, self.x - self.old_x)
        dt0 = angle_diff(t_yaw, self.old_yaw) #Angle to move out radius
        dt1 = angle_normalize(self.yaw - self.old_yaw - dt0) #Final heading of the robot

        return dt0, r, dt1


    def setOld(self):
        """
        Update old values for finding the change in position and heading
        """
        self.old_x = self.x
        self.old_y = self.y
        self.old_yaw = self.yaw


    def checkLaser(self, msg):
        """
        When triggered, find the closest distance in the laser scan

        msg - LaserScan rosmsg
        """
        if self.laser_flag:
            ranges = np.array(msg.ranges)
            ranges[np.where(ranges==0.0)[0]] = np.inf
            self.min_index = np.argmin(ranges)
            self.closest_dist = ranges[self.min_index]
            self.laser_flag = False


if __name__ == "__main__":
    rospy.init_node('manage_sensors')
    sensor_manager = SensorArray()
    rate = rospy.Rate(5)

    #Debugging
    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest
        # map to odom transform

        #
        while sensor_manager.old_x is None:
            continue
        dto, r, dt01 = sensor_manager.getDelta()
        print((dto,r,dt01))

        if r > .5:
            sensor_manager.laser_flag = True
            sensor_manager.setOld()
            while(sensor_manager.laser_flag):
                continue
            print("Update partciles!")
            print('NEW LASER' + str(sensor_manager.closest_dist))

            # self.particle_manager.deleteParticles((dto, r, dt01), self.sensor_manager.closest_dist, self.occupancy_field)
            # self.particle_manager.addParticles()

        rate.sleep()
