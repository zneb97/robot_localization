#!/usr/bin/env python

"""
Ben Ziemann / Nick Steelman
Last updated: 10/3/18

Trigger updates to particles to determine location of the robot
Interface with Rviz for visualization
"""

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from sensorArray import SensorArray
from particle_manager import ParticleManager
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from std_msgs.msg import Header


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()

        self.sensor_manager = SensorArray()
        self.particle_manager = ParticleManager()
        rospy.init_node('pf')

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        #2D Location estimator button from RVIZ
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)



        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project


        #Difference in position to trigger particle update
        self.movement_threshold = .1


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI. From 2D Position locator button """

        #This will be in map frame
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)


        # TODO this should be deleted before posting
        # self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
        #                                                 msg.header.stamp)

        # initialize your particle filter based on the xy_theta tuple
        # self.particle_manager.initParticlesUniform(self.occupancy_field)
        self.sensor_manager.laser_flag = True
        print('Entering While')

        while(self.sensor_manager.laser_flag):
                    print('Im going')
                    rospy.sleep(.5)
                    continue#wait for confirmation that we have update laser scan
        print('EXITING While')
        self.particle_manager.initParticleHeading(xy_theta, self.occupancy_field, self.sensor_manager.min_index)
        # send an initial pose array
        poseArray = PoseArray(header = Header(seq = 10, stamp = rospy.get_rostime(), frame_id = 'map'))
        for particle in self.particle_manager.current_particles:
            poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particle[0], particle[1], particle[2]))
        self.particle_pub.publish(poseArray)


    def run(self):
        '''This is the main loop to continuously run localization on the
        movements of the robot'''
        rate = rospy.Rate(5)

        #Ensure particles have been placed
        print("first while")
        while self.sensor_manager.old_x is None or (not len(self.particle_manager.current_particles)):
            
            print('Old x ',self.sensor_manager.old_x)
            print('Len PM ',len(self.particle_manager.current_particles))
            rate.sleep()
            continue
        # Main Loop
        print("there are particles now")
        while not(rospy.is_shutdown()):

            #See how the robot has moved
            dto, r, dt01 = self.sensor_manager.getDelta()
            print("inside main loop")
            if r > self.movement_threshold: # if the robot moved greater than the desired amount
                print("Moved enough, updating")
                #Tell the callback we want the current minimum of the laser scan
                self.sensor_manager.laser_flag = True
                #reset pointer to last position we updated
                self.sensor_manager.setOld()
                while(self.sensor_manager.laser_flag):
                    continue#wait for confirmation that we have update laser scan
                print("Flag broken")
                print(self.sensor_manager.closest_dist)
                #Keep the most relevant particles
                self.particle_manager.deleteParticles((dto, r, dt01), self.sensor_manager.closest_dist, self.occupancy_field)
                # #Assign more particles
                self.particle_manager.addParticlesHeading(self.occupancy_field,self.sensor_manager.min_index)

                #Send current particles via publisher
                poseArray = PoseArray(header = Header(seq = 10, stamp = rospy.get_rostime(), frame_id = 'map'))
                for particle in self.particle_manager.current_particles:
                    poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particle[0], particle[1], particle[2]))
                self.particle_pub.publish(poseArray)
                print("Poses published")
               # print('Time Left: ', rate.remaining()) #are we over computing for our rate


            #update map position
            self.transform_helper.send_last_map_to_odom_transform()
            rate.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
