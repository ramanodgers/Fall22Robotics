#!/usr/bin/env python3


# TODO: final python file that uses localization and path_finder to move the objects accordingly 
from path_finder import PathFinder
from distutils.log import ERROR
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import rospy
import moveit_commander
import math
import cv2
import os
import utils
import path_finder
import time 
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


class MoveRobot(path_finder.PathFinder): 
    def __init__(self):
        #rospy.init_node('turtlebot3_move_robot')

        path_finder.PathFinder.__init__(self)
        rospy.sleep(5)
        self.initialized = False
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        #TODO: this should be altered
        self.colors = ["pink", "orange", "green", "yellow"]
        # TODO: change these to actual locations 
        self.locations = {1: (0, 0), 2: (1, 1), 3: (2, 2), 4: (3, 3), 5: (4, 4), 6: (5, 5), 7: (6, 6), 8: (7, 7), 9: (8, 8), 10: (9, 9)}
        
        # self.move_arm = moveit_commander.MoveGroupCommander("arm")
        # self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        # # Reset arm position
        # self.move_gripper.go([0.019, 0.019], wait=True)
        # self.move_gripper.stop()
        # self.move_arm.go([0,0,0,0], wait=True)
        self.utils = utils.Utils(10)

        self.obj_in_cam = [0] * 10
        rospy.sleep(2)
        print('move')

        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.sleep(3)
        move = Twist()
        move.angular.z = 0.1
        self.cmd_publisher.publish(move)
        #self.utils.move(0,0.5)
        rospy.sleep(30)
        move.angular.z = 0
        self.cmd_publisher.publish(move)
        #self.utils.move(0,0)
        self.move_to_dest()



    def user_object_and_dest(self):
        color = input("What color would you like to move?\nOptions: Pink, Orange, Green, Yellow")
        color = color.lower()
        destNum = int(input("which destination spot would you like to move the object to?"))
        if destNum not in self.locations:
            print("wrong input try again and input a number between 1 and 10")
        dest = self.locations[destNum]
        return color, dest

    def set_curr_col_and_dest(self):
        self.color, self.dest = self.user_object_and_dest()

    def find_object(self, color, speed):
        res = self.utils.get_color_center(color)
        self.obj_in_cam.append(res)

        # The case when the tude does not appear in the images
        # in any two of the consecutive frames
        if np.mean(self.obj_in_cam[-5:-1]) < 0.3:
            self.utils.move(0, 0.3)
            print("Looking for tube")
        # The case when the tude appear in two of the consecutive frames
        # we decide that we've found the tube
        else:
            h, w, d = self.utils.image.shape
            err = w / 2 - self.utils.color_center
            self.utils.move(speed, err * 0.002)
            print("Obj in image")

    def face_object(self, tar_dist):
        self.utils.get_closest_dir()
        self.utils.get_closest_dist()
        self.utils.move(self.utils.err_proportional_dist(tar_dist, self.utils.closest_dist, 0.2),\
                self.utils.err_proportional_dir(0, self.utils.closest_dir, 20))

    def move_towards_obj(self, tar_dist, color):
        self.obj_in_cam = [0] * 10
        for i in range(300):
            self.find_object(color, 0.15)
            if self.utils.get_closest_infront(10) < 0.2 and np.mean(self.obj_in_cam[-5:-1]) > 0.9:
                break
            rospy.sleep(1)
        
        for i in range(100):
            self.face_object(tar_dist)
            rospy.sleep(1)
        self.utils.stop()

    def pick_up_object(self, color, tar_dist):
        self.move_towards_obj(tar_dist, color)
        # TODO: move arm to pickup obj

    def move_to_dest(self):
        i = 0

        #self.start = self.real_to_cell(self.robot_estimate)
        #self.end = (self.start[0]+5, self.start[1])
        self.aStar()
        rospy.sleep(5)
        print(self.linear_path)

        while i < len(self.linear_path)-1:
            self.move_to_next_point(self.linear_path[i],self.linear_path[i+1])
            i+=1
        self.utils.stop()
        # TODO: has object picked up and moves it to the inputted destination
        pass
    def drop_object(self):
        # TODO: has reached destination and drops off object
        pass
    def move_to_objects(self):
        # TODO: robot moves back to objects and repeats process
        pass
    def run(self):
        pass

    def move_to_next_point(self,point1,point2):


        origin = self.real_to_cell(self.robot_estimate)

        deltax = origin[0]-point2[0]
        deltay = origin[1]-point2[1]
        hypotenuse = math.sqrt(deltax**2 +deltay**2)

        desired_yaw = math.atan(deltay/deltax)
        deltayaw = desired_yaw - get_yaw_from_pose(self.robot_estimate)

        while abs(deltayaw) > 0.2 or hypotenuse >2:
            if not self.new_pose:
                continue

            print('moving thing')
            origin = self.real_to_cell(self.robot_estimate)

            deltax = origin[0]-point2[0]
            deltay = origin[1]-point2[1]

            desired_yaw = math.atan(deltay/deltax)
            deltayaw = desired_yaw - get_yaw_from_pose(self.robot_estimate)
            hypotenuse = math.sqrt(deltax**2 +deltay**2)
            time= abs(deltayaw)*4
            time2 = hypotenuse *6

            rotation = deltayaw/time
            speed = hypotenuse/time2

            self.utils.move(speed,rotation)

            self.new_pose = False

        hypotenuse = math.sqrt(deltax**2 +deltay**2)
        time = hypotenuse

if __name__=="__main__":
    

    MoveRobot = MoveRobot()



    rospy.spin()











