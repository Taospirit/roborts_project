#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import random
import copy
import numpy as np
# import google.protobuf as pb
import rospy
import rospkg
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# from pi_trees_ros.pi_trees_ros import *
from Battle_tmp import BattleEnv
from controller import Controller
from decision.msg import EnemyPos
from roborts_msgs.msg import GimbalAngle, GimbalRate, ShootInfo, ShootState, TwistAccel

Point_goal = {'x':4.0, 'y':4.5, 'theta':-90}
location_num = 0

class TwistControl():
    def __init__(self, move_x, move_y, move_z, angular_z):
        self.Twist = Twist()
        self.Twist.linear.x = move_x
        self.Twist.linear.y = move_y
        self.Twist.linear.z = move_z
        self.Twist.angular.x = 0
        self.Twist.angular.y = 0
        self.Twist.angular.z = angular_z

class Search():
    def __init__(self, blackboard=None):
        # super(Search, self).__init__(name)
        # self.name = name
        # self.blackboard = blackboard
        # self.location_num = 0
        self.closed_list = []
        self.search_order = [0, 1, 2, 3]
        self.search_enemy = False
        self.rotation_theta_threshold = 30

    def run(self):        
        location_num = self.getLocationNum()
        if len(self.closed_list) >= len(env.search_regions[location_num]):
            location_num += 1
            location_num %= len(env.search_regions)
            rospy.logwarn("region {} is clear, going to region {}".format(location_num-1, location_num))
            self.closed_list[:] = []

        goal_x, goal_y, goal_theta = self.getPointToGo(env.search_regions[location_num])

        if not env.isActionAvaliable(goal_x, goal_y, goal_theta):
            rospy.logwarn('cannot goto the point')
            self.closed_list.append([goal_x, goal_y, goal_theta])
        else:
            if self.search_enemy:
                if abs(goal_theta - env.robot_pose['theta']) > self.rotation_theta_threshold:
                    rospy.logwarn('Rotation for point to enemy')
                    env.TwitsRotation(goal_theta)
                rospy.logwarn('Goto enemy point {}, {} with {}'.format(goal_x, goal_y, goal_theta))
            else:
                rospy.loginfo('Goto search point {}, {} with {}'.format(goal_x, goal_y, goal_theta))

            env.SEARCH_cancel_flag = 0
            env.send_goal_in_search(env.navgoal)

            while env.SEARCH_cancel_flag == 1 and env.is_blocked:
                env.getEscaspeDirection()
                rospy.logwarn('try to escaspe!')
                ctrl.send_vel(TwistControl(env.escape_v_x, env.escape_v_y, 0, 0).Twist)

            if env.SEARCH_cancel_flag == 2: # supply needed out
                return True

            if env.SEARCH_cancel_flag == 3: # close out
                return False

            if env.SEARCH_cancel_flag == 4: # detect and refresh enemy position
                return True

            if env.SEARCH_cancel_flag == 5: # not detect and hit rotation
                self.getEnemyDirection()
                env.TwitsRotation(env.twist_goal_theta)
                env.ARMOR_HIT_NUM = -1
                return True

            if env.SEARCH_cancel_flag == 6: # search points smoothly
                if self.search_enemy:
                    self.closed_list[:] = []
                else:
                    self.closed_list.append([goal_x, goal_y, goal_theta])
                return True            
        return True

    def getLocationNum(self): # get region number robot in
        if env.robot_pose['y'] < 2.5:
            if env.robot_pose['x'] < 3.5:
                return 0
            else:
                return 3
        elif env.robot_pose['x'] < 4.5:
            return 1
        else:
            return 2

    def getPointToGo(self, open_list):
        # detecte event
        if env.detection_result:
            self.search_enemy = True
            point_theta = math.degrees(math.atan2(env.enemy_position['y'] - env.robot_pose['y'], env.enemy_position['x'] - env.robot_pose['x']))
            return env.enemy_position['x'], env.enemy_position['y'], point_theta
        self.search_enemy = False
        # special event
        if abs(int(env.ROBOT_ID)-3) < 2 and env.BLUE_BONUS_STATUS == 1: # 己方红，敌方蓝
            # TODO: choose appropriate point to enemy buff area from self pose            
            pass
        if abs(int(env.ROBOT_ID)-13) < 2 and env.RED_BONUS_STATUS == 1: # 己方蓝，敌方红
            pass
        # choose search point
        for p in open_list:
            if p in self.closed_list: 
                continue
            else:
                return p[0], p[1], p[2] # return x, y, theta
        # TODO: more effective way to search when lose enemy in a region 

if __name__ == '__main__':
    rospy.init_node('search_test')
    rate = rospy.Rate(50)
    ctrl = Controller()
    env = BattleEnv()
    SEARCH = Search()
    # tflistener = tf.TransformListener()
    ctrl.global_path_planner_action_client.wait_for_server(rospy.Duration(0.5))
    ctrl.local_path_planner_action_client.wait_for_server(rospy.Duration(0.5))

    rospy.loginfo('Start the ICRA_RM!!!!!!')
    STATE = True
    while not rospy.is_shutdown():
        if STATE:
            print ("\n******************ENTER INTO SEARCH********************")
            STATE = SEARCH.run()
            rate.sleep()
        
        else:
            print ("******************ENTER INTO FOLLOW******************")
            break