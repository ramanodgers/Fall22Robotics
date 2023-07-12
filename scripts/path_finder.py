#!/usr/bin/env python3

import particle_filter
from nav_msgs.msg import OccupancyGrid
import rospy
from geometry_msgs.msg import Twist

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
import time

from queue import PriorityQueue
from particle_filter import Particle

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

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

class PathFinder(particle_filter.ParticleFilter):
    def __init__(self):
        #this inherits a modified particle filter, including its node. 

        particle_filter.ParticleFilter.__init__(self)
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.path_cloud=[]
        self.old_yaw = 0

        self.map = OccupancyGrid()

        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        #converts to cell coordinates
        grid_x = self.map.info.width
        self.width = grid_x
        grid_y = self.map.info.width
        self.height = grid_y

        self.resolution = self.map.info.resolution
        
        #placeholder start
        self.end = (230, 176)
        self.start = (250, 176)

        self.new_unoccupied = []
        self.linear_path = []

        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)

        # this moves the robot forward until it receives 4 pose estimates from particle_filter, where the 
        # new self.counter variable has been added
        move = Twist()
        move.linear.x = 0.02
        print('initial move ')
        self.cmd_publisher.publish(move)
        self.counter = 0
        while self.counter <4:
            continue
        move.linear.x = 0
        print('stopping')
        self.cmd_publisher.publish(move)

        print(self.real_to_cell(self.robot_estimate))
        self.move_to_dest()

    def move_to_dest(self):
        # this function runs aStar() and iterates through each point by calling move_to_next_point

        self.start = self.real_to_cell(self.robot_estimate)
        self.aStar()
        self.old_yaw =-1

        i = 0
        while i < len(self.linear_path)-1:
            self.move_to_next_point(self.linear_path[i],self.linear_path[i+1], i)
            i+=1
        print('ENDED')
        pass

    def move_to_next_point(self,point1,point2, j):
        print('NEXTPOINT', end = ' ')
        print(j)

        #remnants of odometry work
        #saved_pose = self.odom_pose.pose

        saved_local = self.robot_estimate
        origin = self.real_to_cell(saved_local)


        # For travelling to the first point, the robot orients itself based on its localisation estimate
        # After that, it simply takes the difference between its assumed pose and the next path point
        if j==0:
            deltax = point2[0]-origin[0]
            deltay = point2[1]-origin[1]
            self.old_yaw = get_yaw_from_pose(saved_local)
            if abs(deltax) > 0.00001:
                desired_yaw = math.atan(deltay/deltax)
            else: 
                if deltay > 0:
                    desired_yaw =0
                else:
                    desired_yaw = math.pi
        elif True:
            deltax = point2[0]-point1[0]
            deltay = point2[1]-point1[1]
            if abs(deltax) > 0.00001:
                desired_yaw = math.atan(deltay/deltax)
            else: 
                if deltay > 0:
                    desired_yaw =0
                else:
                    desired_yaw = math.pi

        # this block is the work I did to manually calculate the desired robot orientation
        # when it seemed like the above path did not work 
        '''
        else:
            deltax = point2[0]-point1[0]
            deltay = point2[1]-point1[1]

            if abs(deltax) < 0.00001:
                desired_yaw = 0 
                if deltay < 0:
                    desired_yaw = math.pi
            elif abs(deltay) <0.0000001:
                
                desired_yaw = -math.pi/2 
                if deltax < 0:
                    desired_yaw = math.pi
            else:
                if deltax>0:
                    if deltay>0:
                        desired_yaw = -math.pi/8
                    else:
                        desired_yaw = -3*math.pi/8
                elif deltax<0:
                    if deltay>0:
                        desired_yaw = math.pi/8
                    else:
                        desired_yaw = 3*math.pi/8
        '''
        # remnants of odometry work
        #while (abs(deltayaw)-abs((get_yaw_from_pose(saved_pose) - get_yaw_from_pose(self.odom_pose.pose)))) > 0.15:

        #here the robot rotates in proportion to deltayaw
        deltayaw = desired_yaw - self.old_yaw
        self.old_yaw = desired_yaw
        move = Twist()
        move.angular.z = 0.09 *np.sign(deltayaw)
        move.linear.x = 0
        self.cmd_publisher.publish(move)
        print('turning')
        time.sleep(abs(deltayaw/0.09))

        #Then the robot drives forward a fixed distance because we know the path points are equidistant
        move.angular.z = 0.
        move.linear.x = 0.03
        self.cmd_publisher.publish(move)
        time.sleep(2.2)

        # stops
        move.angular.z = 0.
        move.linear.x = 0
        self.cmd_publisher.publish(move)


        # this block is the last iteration of my work trying to follow the path using localisation
        '''
        origin = self.real_to_cell(self.robot_estimate)
        print(point2, end = ' ')
        print(origin)

        print('moving')
        print(origin)

        deltax = origin[0]-point2[0]
        deltay = origin[1]-point2[1]
        hypotenuse = math.sqrt(deltax**2 +deltay**2)

        desired_yaw = math.atan(deltay/deltax)
        deltayaw = desired_yaw - get_yaw_from_pose(self.robot_estimate)
        self.new_pose==True
        i=0
        self.ang_mvmt_threshold= self.ang_mvmt_threshold/3
        hypotenuse = 100
        deltayaw = 6

        move = Twist()
        move.angular.z = 0
        move.linear.x = 0.05
        self.cmd_publisher.publish(move)


        while hypotenuse >=2:

            if self.new_pose == False:
                #print('old pose')
                # move = Twist()
                # move.angular.z = 0
                # move.linear.x = 0.05
                # print(speed)
                # print(rotation)
                # self.cmd_publisher.publish(move)
                #if j >1:
                    # print('no pose???')
                    # move = Twist()
                    # move.angular.z = 0
                    # move.linear.x = 0.01
                    # self.cmd_publisher.publish(move)
                    # rospy.sleep(1)

                continue
            print('position', end = '')
            print(origin, end = '')

            origin = self.real_to_cell(self.robot_estimate)

            deltax = point2[0] - origin[0]
            print('deltax', end = '')
            print(deltax)
            deltay = point2[1]-origin[1]
            if deltax < 0.1:
                desired_yaw = 0 
                if deltay < 0:
                    desired_yaw = math.pi
            else:
                desired_yaw = math.atan(deltay/deltax)
            deltayaw = desired_yaw - get_yaw_from_pose(self.robot_estimate)
            hypotenuse = math.sqrt(deltax**2 +deltay**2)

            rotation = deltayaw/24
            speed = 0.01
            if abs(rotation) > 0.03:
                speed = 0
            print('deltayaw, hypotenuse, rotation, speed', end = ' ')
            print(deltayaw, end =' ')
            print(hypotenuse, end = ' ')
            print(rotation, end = ' ')
            print(speed, end = ' ')
            print(i)
            print('DESIRED_YAW , ESTIMATED YAW', desired_yaw, get_yaw_from_pose(self.robot_estimate) )
            move = Twist()
            move.angular.z = rotation
            move.linear.x = speed

            self.cmd_publisher.publish(move)

            self.new_pose = False
            i+=1
        self.ang_mvmt_threshold*=3
        '''
        move.linear.x = 0
        move.angular.z = 0
        self.cmd_publisher.publish(move)

    def real_to_cell(self,pose):
        #returns x,y cell coordinates in a tuple 

        #converts to cell coordinates
        cell_x = (pose.position.x - self.map.info.origin.position.x)/self.resolution
        cell_y = (pose.position.y - self.map.info.origin.position.y)/self.resolution
        return(int(cell_x),int(cell_y))


    def h_score(self, curr_node):
        # A* h scores

        x1, y1 = curr_node
        x2, y2 = self.end
        h = abs(x2-x1) + abs(y2-y1)
        return h
    

    def convert_map_to_graph(self):
        #converts the occupancy grid to a cell coordinate grid

        X = np.zeros((self.map.info.width*self.map.info.height, 2))
        total_unoccupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] == 0.0:
                    total_unoccupied += 1
                X[curr, 0] = i
                X[curr, 1] = j
                curr += 1
        
        unoccupied = np.zeros((total_unoccupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] == 0:
                    unoccupied[curr, 0] = int(i)
                    unoccupied[curr, 1] = int(j)
                    curr += 1

        X = np.zeros((self.map.info.width*self.map.info.height, 2))
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr, 0] = i
                X[curr, 1] = j
                curr += 1
        
        occupied = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    occupied[curr, 0] = i
                    occupied[curr, 1] = j
                    curr += 1
        self.occupied = list(map(tuple, occupied))
        self.unoccupied = list(map(tuple, unoccupied))

        i = 0
        while i < len(self.unoccupied):
            if not(self.occupied_in_radius(self.unoccupied[i])):
                self.new_unoccupied.append(self.unoccupied[i])
            i+=1

    def occupied_in_radius(self, coord):
        # modifies the occupancy map such that any cell to close to a wall is also considered occupied

        cell_radius = 2
        x_low_bound = int(coord[0] - cell_radius)
        x_high_bound = int(coord[0] + cell_radius)
        y_low_bound = int(coord[1] - cell_radius)
        y_high_bound = int(coord[1] + cell_radius)
        for r in range(x_low_bound, x_high_bound+1):
            for c in range(y_low_bound, y_high_bound+1):
                if (r, c) in self.occupied:
                    return True
        return False



    def in_bounds(self, coord):
        #tests if a given point is within the bounds of the map

        if coord[0] < 0 or coord[1] < 0 or coord[0] >= self.width or coord[1] >= self.height:
            return False
        return True

    def fill_path(self, f_scores, g_scores):
        #the bulk of the A* algorithm, filling out g and h scores.

        queue = PriorityQueue()
        queue.put((self.h_score(self.start), self.h_score(self.start), self.start))
        visited = []
        path = {}
        bounds = [(0, 1), (1, 0), (-1, 0), (-1, 0), (-1, -1), (1, 1), (1, -1), (-1, 1)]
        while not queue.empty():

            curr_node = queue.get()[2]
            visited.append(curr_node)
            if curr_node == self.end:
                break
            for coord in bounds:
                new_node = (float(curr_node[0] + coord[0]), float(curr_node[1] + coord[1]))
                if self.in_bounds(new_node) and new_node in self.new_unoccupied:
                    pot_g_score = g_scores[curr_node] + 1
                    pot_f_score = pot_g_score + self.h_score(new_node)
                    if pot_f_score < f_scores[new_node]:
                        g_scores[new_node] = pot_g_score
                        f_scores[new_node] = pot_f_score
                        queue.put((pot_f_score, self.h_score(new_node), new_node))
                        path[new_node] = curr_node

        return path


    def aStar(self):
        # function returns the best path to take for the robot with a start and end pos

        self.convert_map_to_graph()
        g_scores = {}
        f_scores = {}
        for r in range(self.width):
            for c in range(self.height):
                g_scores[(r, c)] = float('inf')
                f_scores[(r, c)] = float('inf')
        g_scores[self.start] = 0
        f_scores[self.start] = self.h_score(self.start)
        path = self.fill_path(f_scores, g_scores)

        best_path = {}
        curr_node = self.end
        while curr_node != self.start:

            best_path[path[curr_node]] = curr_node
            curr_node = path[curr_node]
        self.linear_path = [self.start]
        location=self.start
        while location !=self.end:

            self.linear_path.append(best_path[location])
            location = best_path[location]
        print(self.linear_path)

        # this section converts the path to a PoseArray and publishes it to the visualization
        self.path_cloud = []
        for point in self.linear_path:
            pose = Pose()

            #converts to real coordinates
            grid_x = (point[0] * self.resolution) + self.map.info.origin.position.x
            grid_y = (point[1] * self.resolution) + self.map.info.origin.position.y

            #sets particle locations
            pose.position.x = grid_x
            pose.position.y = grid_y
            pose.position.z = 0.0

            # all pointing up
            pose = get_pose_from_yaw(pose,0)

            # default weights to meet the requirements of the type
            w = 1/self.num_particles
            self.path_cloud.append(Particle(pose, w))

        path_pose_array = PoseArray()
        path_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        path_pose_array.poses

        for part in self.path_cloud:
            path_pose_array.poses.append(part.pose)

        self.particles_pub.publish(path_pose_array)
        return best_path


if __name__=="__main__":


    pathfinder = PathFinder()

    rospy.sleep(3)

    rospy.spin()
