#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import math
import random
import time

import numpy as np
import rospkg
import rospy
import tf
from decision.msg import EnemyPos
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
from roborts_msgs.msg import (GimbalAngle, GimbalRate, ShootInfo, ShootState,
                              TwistAccel)
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from battle_env import BattleEnv
from controller import Controller

class TwistControl():
    def __init__(self, move_x, move_y, move_z, angular_z):
        self.Twist = Twist()
        self.Twist.linear.x = move_x
        self.Twist.linear.y = move_y
        self.Twist.linear.z = move_z
        self.Twist.angular.x = 0
        self.Twist.angular.y = 0
        self.Twist.angular.z = angular_z

class BlockJudge():
    def __init__(self, safe_dist = 0.5, block_dist = 0.8, rate_1 = 4, rate_2 = 40, ratio = 0.5):
        self.safe_dist = safe_dist
        self.block_dist = block_dist
        self.rate_1 = rate_1
        self.rate_2 = rate_2
        self.ratio = ratio
        self.block_obstacle = [] # 阻塞附近最近的障碍物角点坐标
        self.is_blocked = False
        self.block_count = 0 # 计数
        self.pose_x, self.pose_y = [], []
        self.delta_x, self.delta_y = 0, 0
        self.block_delta_x, self.block_delta_y = 0, 0
        self.escape_v_x, self.escape_v_y = 0, 0
        self.speed_x, self.speed_y = 0, 0

        self.robot_pose = {'x':0, 'y':0, 'theta':0}
        self.obstacle_conner = [[1.4, 1.4], [1.4, 2.4], [1.65, 2.4], [1.65, 1.4],
                                [1.2, 3.75], [1.2, 4], [2.2, 4], [2.2, 3.75],
                                [3.5, 2.375], [3.5, 2.625], [4.5, 2.625], [4.5, 2.375],
                                [6.35, 2.6], [6.35, 3.6], [6.6, 3.6], [6.6, 2.6],
                                [5.8, 1], [5.8, 1.25], [6.8, 1.25], [6.8, 1],
                                [4.5, 4], [4.75, 4], [3.25, 1], [4.5, 1]] # 障碍角点坐标
        rospy.Subscriber('robot_pose', Odometry, self.getSelfPoseCallback, queue_size=1)

    def getSelfPoseCallback(self, data):
        self.robot_pose['x'] = data.pose.pose.position.x
        self.robot_pose['y'] = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        angle = euler_from_quaternion((qx,qy,qz,qw))
        self.robot_pose['theta'] = math.degrees(angle[2])
        self.isBlocked()

    def isBlocked(self):
        dist_min = float('inf')
        for p in self.obstacle_conner:
            dist = math.hypot(p[0] - self.robot_pose['x'], p[1] - self.robot_pose['y'])
            if dist < dist_min:
                dist_min = dist
                self.block_obstacle = p

        if dist_min > self.safe_dist: # 设置安全距离, 大于安全距离直接重置参数
            self.block_count = 0
            self.pose_x[:], self.pose_y[:] = [], []
            self.delta_x, self.delta_y = 0, 0
            self.is_blocked = False
            print ('dist_min is {}, bigger than safe_dist-{}'.format(dist_min, self.safe_dist))

        else:
            self.block_count += 1
            if self.block_count % self.rate_1 == 0: # pose update rate: 40
                self.pose_x.append(self.robot_pose['x'])
                self.pose_y.append(self.robot_pose['y'])

            if self.block_count % self.rate_2 == 0:
                for x in self.pose_x:
                    self.delta_x += abs(x - np.mean(self.pose_x))
                for y in self.pose_y:
                    self.delta_y += abs(y - np.mean(self.pose_y))

                mean_delta_x = self.delta_x / len(self.pose_x)
                mean_delta_y = self.delta_y / len(self.pose_y)

                if mean_delta_x < self.block_dist and mean_delta_y < self.block_dist:
                    rospy.logwarn('robot is blocked near point:{}!'.format(self.block_obstacle))
                    self.is_blocked = True
                
    def getEscaspeDirection(self): # 离障碍物太近会堵住无法移动，需要主动摆脱
        # get distance from the closest corner point to the line robot in
        cos_theta, sin_theta = math.cos(math.radians(self.robot_pose['theta'])), math.sin(math.radians(self.robot_pose['theta']))
        line_dist = abs(cos_theta * (self.block_obstacle[1]-self.robot_pose['y']) - sin_theta * (self.block_obstacle[0]-self.robot_pose['x']))
        
        # to make sure sign 
        A = np.mat([[sin_theta, -cos_theta], [-cos_theta, -sin_theta]])
        B = np.mat([[sin_theta * self.robot_pose['x'] - cos_theta * self.robot_pose['y']], [-cos_theta * self.block_obstacle[0] - sin_theta * self.block_obstacle[1]]])
        jion_point = np.linalg.solve(A, B)
        delta_x = jion_point[0] - self.block_obstacle[0]
        delta_y = jion_point[1] - self.block_obstacle[1]
        delta = math.hypot(delta_x, delta_y)
       
        # # escape direction
        # self.block_delta_x = abs(line_dist * sin_theta) * delta_x / abs(delta_x)
        # self.block_delta_y = abs(line_dist * cos_theta) * delta_y / abs(delta_y)

        # escape speed in map coordinates
        self.speed_x, self.speed_y = self.ratio * delta_x / delta, self.ratio * delta_y / delta
        # escape speed in robot coordinates
        self.escape_v_x = self.speed_x * cos_theta + self.speed_y * sin_theta 
        self.escape_v_y = - self.speed_x * sin_theta + self.speed_y * cos_theta

if __name__ == "__main__":
    rospy.init_node('block_test')
    rate = rospy.Rate(20)
    ctrl = Controller()
    bj = BlockJudge()
    k = True
    while not rospy.is_shutdown():
        while bj.is_blocked:
            k = True
            bj.getEscaspeDirection()
            ctrl.send_vel(TwistControl(bj.escape_v_x, bj.escape_v_y, 0, 0).Twist)
            print ('\ntry to escape for map speed_x {}, speed_y {}'.format(bj.speed_x, bj.speed_y))
            print ('robot escape_v_x is {}, escape_v_y is {}'.format(bj.escape_v_x, bj.escape_v_y))
        if k:
            print ('robot escape success in {}, {}!'.format(bj.robot_pose['x'], bj.robot_pose['y']))
            k = False
        rate.sleep()

        
      


        