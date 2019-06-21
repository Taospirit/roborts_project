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
from tf.transformations import quaternion_from_euler

from battle_env import BattleEnv
from controller import Controller


class GotoBuff(py_trees.Behaviour):
    def __init__(self, blackboard=None):
        # super(GotoBuff, self).__init__(name)
        # self.name = name
        # self.blackboard = blackboard
        self.start_buff = 0
        self.buff_point = [6.3, 1.75, 150]
        self.remain_time = 7

    def update(self):
        print ('************************ENTER_GotoBuff**************************')
        # if not self.blackboard.ForceBuff:
        #     env.outDodge()
            # self.blackboard.EnterFollow = False
            # self.blackboard.ForceBuff = False
    
        if env.SELF_BONUS_STATUS == 0:
            rospy.loginfo("BUFF---{}---going to BUFF!!!!".format(env.COLOR))
            if not (self.isInRange(env.robot_pose['x'], 6.1, 6.5) and self.isInRange(env.robot_pose['y'], 1.25, 1.95)):
                if env.isActionAvaliable(self.buff_point[0], self.buff_point[1], self.buff_point[2]):
                    if env.ammo_num <= 16:
                        env.send_goal_force(env.navgoal) # 弹量少时force_send_goal
                    else: # 弹药充足
                        env.BUFF_cancle_flag = 0
                        env.send_goal_buff(env.navgoal)

                        if env.BUFF_cancle_flag == 1:
                            rospy.loginfo('BUFF---{}---BUFF is getting by other robot, cancle goal!'.format(env.COLOR))
                            # return py_trees.Status.FAILURE
                            return False

                        if env.BUFF_cancle_flag == 2:
                            rospy.loginfo('BUFF---{}---detect enemy during goto BUFF, cancle goal!'.format(env.COLOR))
                            # return py_trees.Status.FAILURE
                            return False

                    self.start_buff = rospy.Time.now().secs
                    # self.blackboard.ForceBuff = True
                    env.enterDodge()
                    # return py_trees.Status.SUCCESS
                    return True

            else: # robot have be in BUFF area but BONUS_STATUS is still 0
                stay_time = rospy.Time.now().secs - self.start_buff
                if stay_time < self.remain_time + 2:
                    print ('Buff status is {}, but robot in self buff area aleady! wait for {}'.format(env.SELF_BONUS_STATUS, stay_time))
                    # return py_trees.Status.SUCCESS
                    return True
                # else:
                #     rospy.logwarn('BUFF--------time out')
                #     return py_trees.Status.FAILURE

        elif env.SELF_BONUS_STATUS == 1:
            if self.isInRange(env.robot_pose['x'], 5.8, 6.8) and self.isInRange(env.robot_pose['y'], 1.25, 2.3):
                stay_time = rospy.Time.now().secs - self.start_buff
                if stay_time < self.remain_time:
                    print ('The buff is {},robot is getting buff! wait for {}'.format(env.SELF_BONUS_STATUS, stay_time))
                    # return py_trees.Status.SUCCESS
                    return True
            else:
                rospy.loginfo('The buff is {}, somebody is getting buff, OUT!'.format(env.SELF_BONUS_STATUS))
                # return py_trees.Status.FAILURE
                return True

        rospy.logwarn('BUFF--------TIME OUT----------')
        env.outDodge()
        # self.blackboard.ForceBuff = False
        # return py_trees.Status.FAILURE
        return False
                        
    def isInRange(self, val, val1, val2):
        if val < val1:
            return False
        elif val > val2:
            return False
        return True

if __name__ == '__main__':
    rospy.init_node('buff_test')
    rate = rospy.Rate(50)
    ctrl = Controller()
    env = BattleEnv()
    BUFF = GotoBuff()
    # tflistener = tf.TransformListener()
    ctrl.global_path_planner_action_client.wait_for_server(rospy.Duration(0.5))
    ctrl.local_path_planner_action_client.wait_for_server(rospy.Duration(0.5))

    rospy.loginfo('Start the BUFF TEST')
    STATE = True
    while not rospy.is_shutdown():
        if STATE:
            print ("\n******************ENTER INTO BUFF********************")
            STATE = BUFF.update()
            rate.sleep()
        
        else:
            print ("******************OUT BUFF******************")
            break
