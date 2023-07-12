#!/usr/bin/env python3


# TODO: this file does everything with having the robot know where it is in the maze
# this will be used in the path finding algorithm
import rospy 

import numpy as np
from numpy.random import random_sample, normal
import math
import copy
from random import randint, random, uniform 

#: TODO: NOT SURE IF THIS CAN JUST BE DONE USING A SIMPLE ROS PACKAGE ROBOT_LOCALIZATION / IF /HOW IT WORKS
class Localization():
    def __init__(self):
        self.initalized = False
        rospy.init_node("turtlebot3_localization")
    # all other class methods go here
    def get_curr_pos(self):
        # TODO: using localization get the robots current position and return that coordinate on the map
        return (0, 0)
