#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import sys
import time
import rospy
import numpy as np
from roborts_msgs.msg import TeammateMsg 
from battle_env import BattleEnv

if __name__ == '__main__':
    rospy.init_node('pub_self_msg')
    rate = rospy.Rate(10)
    
    self_env = BattleEnv()
    self_msg_pub = rospy.Publisher('send_to_teammate', TeammateMsg, queue_size=1)
    self_msg = TeammateMsg()
    #std_msgs/Header header
    # uint16 robot_id
    # float64 robot_pos_x
    # float64 robot_pos_y
    # uint8 robot_pjt_info
    # bool find_enemy1
    # float64 enemy1_pos_x
    # float64 enemy1_pos_y
    # bool find_enemy2
    # float64 enemy2_pos_x
    # float64 enemy2_pos_y
    # float64 teammate_pos_x
    # float64 teammate_pos_y
    # uint16 teammate_id
    # uint16 teammate_remain_hp
    # uint8 teammate_pjt_info
    # uint8 cmd
    while not rospy.is_shutdown():
        try:
            print ('pub_self_msg')
             # self_msg.robot_id = self_env.SELF_ID
            self_msg.teammate_id = self_env.MATE_ID # 指定发送队友的ID
            self_msg.robot_pos_x = self_env.robot_pose['x']
            self_msg.robot_pos_y = self_env.robot_pose['y']
            self_msg.robot_pjt_info = self_env.pjt_info

            self_msg.find_enemy1 = self_env.detection_result
            self_msg.enemy1_pos_x = self_env.ENEMY_POSE_TO_MATE['x']
            self_msg.enemy1_pos_y = self_env.ENEMY_POSE_TO_MATE['y']

            # self_msg.mate_supply = self_env.IS_SUPPLY
            self_msg.cmd = self_env.IS_SUPPLY
            self_msg_pub.publish(self_msg)
        except rospy.ROSInterruptException:
            pass
       
        rate.sleep()
