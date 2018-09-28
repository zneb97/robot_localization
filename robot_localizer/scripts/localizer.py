import rospy
import math
from std_msgs.msg import Strinocg
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt


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
        self.closest_dist = None

        #ROS
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
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = angles[2]

        return (pose.position.x, pose.position.y, angles[2])

    def getDelta():
        

    

    def setOld():
        pass


    def checkLaser(self, msg)