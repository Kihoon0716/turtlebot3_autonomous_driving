#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import *
from compressed_image_transport import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf
import math
import cv2
import numpy as np
import threading
from std_msgs.msg import String
import heapq
import timeit
from time import sleep
import tensorflow as tf

from collections import deque
import dqn
from typing import List
import random

pi = np.pi
class Env_manager():
    def __init__(self):

        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty) # reset service
        
        self.action_size = 11
        self.input_size = 37
        self.step_count = 0


    def publishing_vel(self, angular_x, angular_y, angular_z, linear_x, linear_y, linear_z):
        vel = Twist()
        vel.angular.x = angular_x
        vel.angular.y = angular_y
        vel.angular.z = angular_z
        vel.linear.x = linear_x
        vel.linear.y = linear_y
        vel.linear.z = linear_z
        self._pub.publish(vel)
    
    def getState(self, scan):
        scan_range = []
        min_range = 0.13
        done = False

        scan_idx = []
        scan_idx.append(0)
        for i in range(1,19):
            scan_idx.append(i * 5)
            scan_idx.append(359 - i*5)

        for i in scan_idx:
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range):
            done = True

        return scan_range, done

    def setReward(self, state, done, action):
        self.step_count += 1

        right = state[18]
        left = state[36]
        reward = abs(right - left)
        reward = 0
        #print "reward : ", reward
        if done:
            #rospy.loginfo("Collision!!")
            reward = self.step_count - 2000
            self.move(0, 0)
        return reward


    def step(self, action):
        max_angular_vel = 0.3
        #print "action : ", action
        ang_vel = (float(self.action_size - 1)/2 - action) / (self.action_size/2) * max_angular_vel

        speed = 0.22
        self.move(speed, ang_vel)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        reward = self.setReward(state, done, action)

        return np.asarray(state), reward, done 
    def reset(self):
        
        try:
            self.reset_proxy()
            #print "reset"
        except (rospy.ServiceException) as e:
            print "gazebo/reset_simulation service call failed"


        data = None
        while data is None:
            try:   
                #print "wait"
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass
        state, done = self.getState(data)
        return state
    def move(self, speed, direction):
        #print "speed : ", speed, " direction : ", direction
        self.publishing_vel(0, 0, direction, speed, 0, 0)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('maze_pathfinder')
    Maze_pathfinder = Maze_pathfinder()
    Maze_pathfinder.main()
