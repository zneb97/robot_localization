import rospy
import math
from std_msgs.msg import Strinocg
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from math import sin, cos, atan2, pi, fabs

import matplotlib.pyplot as plt

def distance(x,y):
    return np.linalg.norm(np.array(x)-np.array(y))

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return atan2(sin(z), cos(z))

def angle_diff(a, b):
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
        self.old_theta = None
        self.x = None
        self.y = None
        self.theta = None

        self.laser_flag = False
        self.closest_dist = None


        #ROS
        rospy.init_node('manage_sensors')
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)
        rospy.Subscriber("/odom", Odometry, self.setLocation)

    def setLocation(self, msg):
        """
        Convert pose (geometry_msgs.Pose) to a (x, y, theta) tuple
        Constantly being called as it is the callback function for this node's subscription

        odom is Neato ROS' nav_msgs/Odom msg composed of pose and orientation submessages
        """
        pose = odom.pose.pose
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        if self.x is None:
            self.old_x = pose.position.x
            self.old_y = pose.position.y
            self.old_yaw = angles[2]
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = angles[2]


        return (pose.position.x, pose.position.y, angles[2])

    def getDelta(self):
        r = distance((self.old_x,self.old_y),(self.x,self.y))
        t_yaw = atan2(self.y - self.old_y, self.x - self.old_x)
        dt0 = angle_diff(t_yaw, self.yaw)
        dt1 = self.yaw - self.old_yaw - dt0
        return dto, r, dt1

    def setOld(self):
        self.old_x = self.x
        self.old_y = self.y
        self.old_yaw = self.yaw


    def checkLaser(self, msg):
        if self.laser_flag:
            self.closest_dist = np.min(msg.ranges)
            self.laser_flag = False
