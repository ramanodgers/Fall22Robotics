#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample, normal
import math
import copy
from random import randint, random, uniform 

from likelihood_field import LikelihoodField
from measurement_update_likelihood_field import compute_prob_zero_centered_gaussian



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def get_pose_from_yaw(pose,yaw):
    # this function takes in a pose and a new yaw. It outputs the same pose with this updated yaw.
    # (it edits in place but for ease of use we also return the pose)

    pose.orientation = Quaternion()
    q = quaternion_from_euler(0,0,yaw)

    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose

def draw_random_sample(particle_cloud, n):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # This function draws from uniform[0,1] and sums the weights until it is >= the draw,
    # Thereby randomly selecting particles by weight

    weightlist = []
    for part in particle_cloud:
        weightlist.append(part.w)

    ans = []

    for i in range(n):
        draw = uniform(0,1)
        count = 0
        for j, weight in enumerate(weightlist):
            count+=weight
            if count >= draw:
                # resample the particle that the draw landed on
                # we also add noise here.
                new_part = copy.deepcopy(particle_cloud[j])
                new_part.pose.position.x += normal(0,0.002)
                new_part.pose.position.y += normal(0,0.002)
                yaw = get_yaw_from_pose(new_part.pose)
                yaw+= normal(0,0.001)
                new_part.pose = get_pose_from_yaw(new_part.pose,yaw)

                ans.append(new_part)
                break

    return ans 


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w


class ParticleFilter:

    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 1000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.08      
        self.ang_mvmt_threshold = (np.pi / 12)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        rospy.sleep(1)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.likelihood_field = LikelihoodField()

        self.initialized = True
        self.new_pose = False


    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        # First, we randomly search occupancy grid for known points within the map. 
        # We then convert this into cell coordinates and then x,y coordinates.
        # Finally, we randomly generate an orientation for each particle.

        resolution = self.map.info.resolution
        for i in range(self.num_particles):

            pose = Pose()

            # generates known random empty point in occupancy grid
            while True:
                position = randint(0, 384*384-2)
                if self.map.data[position] == 0:
                    break

            #converts to cell coordinates
            grid_x = position%self.map.info.width
            grid_y = position//self.map.info.width

            #converts to real coordinates
            grid_x = (grid_x * resolution) + self.map.info.origin.position.x
            grid_y = (grid_y * resolution) + self.map.info.origin.position.y

            #sets particle locations
            pose.position.x = grid_x
            pose.position.y = grid_y
            pose.position.z = 0.0

            # random orientation
            pose = get_pose_from_yaw(pose,uniform(0,math.pi*2))

            # normalised weights
            w = 1/self.num_particles
            self.particle_cloud.append(Particle(pose, w))

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # sums weight of all particles, divides all weights by that sum. 

        total_sum = 0
        for part in self.particle_cloud:
            total_sum += part.w
        for part in self.particle_cloud:
            part.w = part.w/total_sum



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.new_pose = True
        self.counter +=1
        
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        #wrapper function to resample particles and then normalise weights.

        self.particle_cloud = draw_random_sample(self.particle_cloud, self.num_particles)
        self.normalize_particles()

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out
                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()

                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # simple average of particles to produce an updated robot estimate.

        #produce average
        x_sum  = 0
        angle_sum = 0
        y_sum = 0
        for part in self.particle_cloud:
            x_sum += part.pose.position.x *part.w
            y_sum += part.pose.position.y *part.w

            yaw = get_yaw_from_pose(part.pose)
            angle_sum+=yaw

        x_sum = x_sum/self.num_particles
        y_sum = y_sum/self.num_particles
        angle_sum = angle_sum/self.num_particles

        # implement new average
        quat_o = quaternion_from_euler(0,0,angle_sum)

        self.robot_estimate.orientation.x = quat_o[0]
        self.robot_estimate.orientation.y = quat_o[1]
        self.robot_estimate.orientation.z = quat_o[2]
        self.robot_estimate.orientation.w = quat_o[3]
        self.robot_estimate.position.x = x_sum *self.num_particles
        self.robot_estimate.position.y = y_sum *self.num_particles

        return


    
    def update_particle_weights_with_measurement_model(self, data):
        # Update the particle weights with laserscan localisation 
        # implemented the likelihood fields range finder algorithm from class

        for part in self.particle_cloud:
            q=1
            k = 0

            while k < len(data.ranges):
                if k> 170 and k <190:
                    pass
                elif data.ranges[k] != 0 and data.ranges[k] < 3.5 :
                    yaw = get_yaw_from_pose(part.pose)

                    x = part.pose.position.x + data.ranges[k]*math.cos(yaw+math.radians(k))
                    y = part.pose.position.y + data.ranges[k]*math.sin(yaw+math.radians(k))

                    dist = self.likelihood_field.get_closest_obstacle_distance(x,y)

                    q=q*(compute_prob_zero_centered_gaussian(dist,0.1))
                k+=36
            part.w = q

        return


        

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        # implemented the algorithm show in Class meeting 5 

        if self.particle_cloud:
            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
            x_change = curr_x - old_x
            y_change = curr_y - old_y
            yaw_change = curr_yaw - old_yaw

            for part in self.particle_cloud:

                a1 = 0.005
                a2 = 0.005
                rot1 = math.atan2(y_change, x_change) - old_yaw
                d = math.sqrt((x_change)**2 + (y_change)**2)
                rot2 = curr_yaw - old_yaw - rot1

                # here we added noise to the rotations and distance values for the particles
                rot1 -= normal(a1*rot1 + a2*d)
                d -= normal(a1*d + a2*(rot1 + rot2))
                rot2 -= normal(a1*rot2 + a2*d)

                part_yaw = get_yaw_from_pose(part.pose)
                new_yaw = part_yaw + rot1 + rot2

                part.pose.position.x += d*(math.sin(part_yaw + rot1)) * self.map.info.resolution
                part.pose.position.y += d*(math.sin(part_yaw + rot1)) * self.map.info.resolution

                part.pose = get_pose_from_yaw(part.pose, new_yaw)


if __name__=="__main__":
    

    pf = ParticleFilter()
    print('partifcle')
    rospy.spin()









