#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import sys
import time
from random import uniform

import actionlib
import rospy
import tf
from apriltags2_ros.msg import AprilTagDetectionArray
from decision.msg import EnemyPos
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               Quaternion, Twist, PointStamped)
from nav_msgs.msg import Odometry
from PIL import Image

from roborts_msgs.msg import (ArmorDetection, ArmorDetectionAction,
                              ArmorDetectionGoal, BonusStatus, GameResult,
                              GameStatus, GameSurvivor, GimbalAngle,
                              GimbalRate, GlobalPlannerAction,
                              GlobalPlannerFeedback, GlobalPlannerGoal,
                              HeatControl, LocalPlannerAction,
                              LocalPlannerGoal, ProjectileInfo,
                              ProjectileSupply, RobotBonus, RobotDamage,
                              RobotHeat, RobotShoot, RobotStatus, ShootInfo,
                              ShootState, SupplierStatus, TwistAccel, TeammateMsg, 
                              Debug, SupplyDistance)
from roborts_msgs.srv import FricWhl, GimbalMode, ShootCmd
from std_msgs.msg import Bool
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
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

class BattleEnv():
    #region-------BASE_INIT--------#
    def __init__(self):    
        self.map = np.array(
            Image.open("{}/icra2019_s.pgm".format(
                os.path.split(os.path.realpath(__file__))[0])))
        self.Map_Width = 1600
        self.Map_Height = 1000

        self.ctrl = Controller()
        self.navgoal = GlobalPlannerGoal()
        self.navgoal.goal.header.frame_id = 'map'
        self.gimbal_angle = GimbalAngle()
        self.gimbal_rate = GimbalRate()
        self.shoot_info = ShootInfo()
        self.shoot_state = ShootState()
        self.twist_accel = TwistAccel()
        self.armor_detection_result = ArmorDetection()
        self.tf_pose = PoseStamped()
        self.global_tf_pose = PoseStamped()
        self.tf_tansformer = TransformListener()
        self.quatarnion = Quaternion()
        self.enemy_pose_robot_frame = EnemyPos()
        self.enemy_pose = EnemyPos()
        self.shoot_cmd = ShootCmd()
        
        self.current_gimbal_mode = -1
        self.current_chassis_mode = -1

        self.shoot_ammo_num = 10
        self.ammo_num = 40 # 弹量
        self.ammo_available_num = 1
        self.num_detected_enemy = 0
        self.num_not_detected_enemy = 0
        
        self.detection_result = False
        self.detection_result_stable = False
        self.detection_result_stable_num = 30
        # self.sudden_find_change_num = 10

        self.team_msg_pub = rospy.Publisher('send_to_teammate', TeammateMsg, queue_size=1)
        self.team_msg = TeammateMsg()
        self.point_pub = rospy.Publisher('enemy_positioin', PointStamped, queue_size=1)
        self.point_pub_ = PointStamped()
        self.point_pub_.header.frame_id = 'map'
        self.point_pub_.header.stamp = rospy.Time()
        self.point_pub_.point.x, self.point_pub_.point.y, self.point_pub_.point.z = 0, 0, 0

        self.robot_pose = {'x': 1, 'y': 1, 'theta': 0}
        self.enemy_position = {'x':0, 'y':0, 'theta':0}
        self.ENEMY_POSE_FROM_MATE = {'x': 0, 'y': 0}
        self.ENEMY_POSE_TO_MATE = {'x': 0, 'y': 0}
        self.enemy_pose.enemy_dist = float('inf')
        self.enemy_last = []
        self.robot_last = []
        self.detect_length = 10 # 保留10帧
        self.enemy_yaw_chassis = 0
        self.gimbal_angle_yaw = 0
        

        self.behavior_mode = -1
        # self.gimbal_mode = -1
        # self.chassis_mode = -1
        self.shoot_mode = 0

        # path planner action to execute the navigation goal
        self.global_path_planner_action_client = actionlib.SimpleActionClient(
            'global_planner_node_action', GlobalPlannerAction)
        self.local_path_planner_action_client = actionlib.SimpleActionClient(
            'local_planner_node_action', LocalPlannerAction)
        self.local_path_planner_goal = LocalPlannerGoal
        self.is_cancel_goal = 0

        # armor detection action
        self.armor_detection_action_client = actionlib.SimpleActionClient(
            'armor_detection_node_action', ArmorDetectionAction)
        self.armor_detection_action_client.wait_for_server(rospy.Duration(0.5))
        self.armor_detection_goal = ArmorDetectionGoal  


        self.supplytalker = rospy.Publisher('projectile_supply', ProjectileSupply, queue_size=1)

        # supply parameters
        self.pjt_info = 0
        self.supply_count = 0
        self.MATE_SUPPLY = 0

        #FIXME:----fix----2019.05.22
        self.SELF_SUPPLY = False
        # ----fix----
        
        self.supply_cooldown = -1
        self.supply_needed = False
        self.supply_amount = 50
        self.supplypoint_list = [[3.7, 3.6, -90], [3.8, 3.6, -90]]
        self.supplypoint_default = [4.15, 4.3, -90]
        self.sendgoalresult = False
        #-----fix------2019.05.30
        self.go_point_num = 2
        #-----fix-----
        self.breaksupply = False
        self.breakpoint = [2.7, 2.5, 0]
        self.tag_info = {'tag_detected': False, 'x': 0, 'z': 0, 'pitch': 0, 'distance': 0 }

        self.buff_needed = True
        self.gotobuff_count = 0
        self.buff_sendgoalresult = True
        self.buff_countflag = True
        self.buff_counttime = 0

        #region---------FOLLOW_INIT----------#
        self.enemy_follow_x = 0
        self.enemy_follow_y = 0
        self.FOLLOW_stop_dist = 0.8 # 实车与仿真设置距离不同
        self.FOLLOW_cancel_flag = 0
        self.beated_num = 0
        self.beated_num_two_secs = 0
        self.detected_by_front = False
        self.detection_result_front = False
        self.detection_result_front_stable = False
        self.num_not_detected_enemy_front = 0
        self.num_detected_enemy_front = 0
        # 左下 左上 中 右上 右下 中上 中下
        self.obstacle_conner = [[1.4, 1.4], [1.4, 2.4], [1.65, 2.4], [1.65, 1.4],
                                [1.2, 3.75], [1.2, 4], [2.2, 4], [2.2, 3.75],
                                [3.5, 2.375], [3.5, 2.625], [4.5, 2.625], [4.5, 2.375],
                                [6.35, 2.6], [6.35, 3.6], [6.6, 3.6], [6.6, 2.6],
                                [5.8, 1], [5.8, 1.25], [6.8, 1.25], [6.8, 1],
                                [4.5, 4], [4.75, 4], [3.25, 1], [4.5, 1]]
        #endregion--------FOLLOW_INIT-----------#   


        #region------BUFF INIT------#
        #讲检测结果放松，现在这个值挺严格
        self.BUFF_detect_num = 20
        self.BUFF_cancle_flag = 0
        self.robot_name = -1 # 0在前30秒补弹，1在后30秒补弹

        #endregioin------BUFF INIT------#
        self.SUPPLY_cancle_flag = 0

        #region---------SEARCH_INIT----------#
        self.twits_goal_x = 0
        self.twits_goal_y = 0
        self.twist_goal_theta = 0

        self.block_obstacle = []
        self.pose_x = []
        self.pose_y = []
        self.delta_x = 0
        self.delta_y = 0
        self.is_blocked = False
        self.block_count = 0
        self.block_delta_x = float('inf')
        self.block_delta_y = float('inf')
        self.escape_v_x = 0 # escape speed when robot blocked in obstacle
        self.escape_v_y = 0
        self.enemy_buff_point = [1.7, 3.25] # enemy buff center in map
        self.search_buff_point = [[0.75, 2.5, 60], [2.5, 2.0, 120], [0.5, 3.5, 0], [3.0, 4.0, -150], [6.0, 4.0, -160], [5.5, 3.0, 180], [4.0, 1.5, 135]]

        self.SEARCH_hurt_flag = False
        self.SEARCH_cancel_flag = 0
        self.SEARCH_cancel_dist = 0.3 # 用于search中平滑移动
        self.SEARCH_cancel_ammo_num = 10
        self.SEARCH_stop_dist = 1.5 # 检测
        self.SEARCH_answer_enemy_buff = False

        self.SEARCH_cancel_remain_hp_rate = 0.7 # precent

        self.SEARCH_status = []
        self.SEARCH_flag = []

        self.search_regions = []


        #FIXME: -----fix-----2019.05.22
        self.search_region_1 = [[2.5, 1.5, 25], [2.5, 2.3, 45]]
        self.search_region_2 = [[2.7, 3.2, 40], [4.1, 3.3, -35]]
        self.search_region_3 = [[5.5, 3.7, -150], [5.5, 2.7, -145]]
        self.search_region_4 = [[5.3, 1.8, 180], [3.9, 1.7, 145]]
        #-----fix-----2019.05.22


        # self.search_region_1 = [[2.8, 1.5, 70], [2.0, 0.7, 60], [1.0, 0.7, 90], [0.7, 2.0, 70]]
        # self.search_region_2 = [[1.0, 3.0, 0], [2.5, 3.2, -60], [4.0, 3.4, 180]]
        # self.search_region_3 = [[5.5, 3.5, -90], [6.0, 4.3, -120], [7.2, 3.3, -90]]
        # self.search_region_4 = [[7.0, 2.0, 180], [5.5, 1.8, 120], [4.0, 1.6, 0]]

        self.search_regions.append(self.search_region_1)
        self.search_regions.append(self.search_region_2)
        self.search_regions.append(self.search_region_3)
        self.search_regions.append(self.search_region_4)
        #endregion--------SEARCH_INIT-----------#


        #region-----------REFEREE_SYSTEM_PARAMETERS_INIT------------# 
        #bonus zone status
        self.RED_BONUS_STATUS = 0
        self.BLUE_BONUS_STATUS = 0

        self.SELF_BONUS_STATUS = 0
        self.ENEMY_BONUS_STATUS = 0
        self.COLOR = 'NONE'

        #game result
        # uint8  DRAW=0
        # uint8  RED_WIN=1
        # uint8  BLUE_WIN=2
        # uint8  result

        #game status
        self.GAME_STATUS = 0 # 初始为0,修改仅测试用
        self.REMAINING_TIME = float('inf')

        #robot survival
        self.SURVIVOR_LIST = {3: True, 4: True, 13: True, 14: True}

        #projectile supply
        self.PROJECTILE_SUPPLY = 0

        #robot bonus
        self.ROBOT_BONUS = False

        #robot damage
        self.ARMOR_HIT_NUM = -1 # 

        #robot power and heat data
        self.CHASSIS_VOLT = 0
        self.CHASSIS_CURRENT = 0
        self.CHASSIS_POWER = 0
        self.CHASSIS_POWER_BUFFER = 222
        self.SHOOTER_HEAT = 10000

        #robot shoot data
        self.SHOOT_FREQUENCY = 0
        self.SHOOT_SPEED = 0

        #robot status
        # self.ROBOT_STATUS = data 
        self.SELF_ID = 0 # 3/4 for red3/red4, 13/14 for blue3/blue4
        self.REMAIN_HP = 2000
        self.MAX_HP = 2000
        self.HEAT_COOLING_LIMIT = 0
        self.HEAT_COOLING_RATE = 0
        self.GIMBAL_OUTPUT = False
        self.CHASSIS_OUTPUT = False
        self.SHOOTER_OUTPUT = False

        #supplier status
        self.SUPPLIER_STATUS = 0

        #region---------TEAM_MSG---------#
        self.ID_MAP = {3:4, 4:3, 13:14, 14:13}
        self.CMD_TO_MATE = 0
        self.MATE_ID = 0
        self.MATE_POSE = {'x':0, 'y':0, 'theta':0}
        self.MATE_REMAIN_HP = 0
        self.MATE_PJT_INFO = 0 # 0 for none, 1 for not enough, 2 for enough
        self.CMD_FROM_MATE = 0 # 0 for normal, 1 for cooperation, 2 for help
        self.CMD_TO_MATE = 0
        self.IS_SUPPLY = 0
        #endregion---------TEAM_MSG----------#

        self.rotate_at_begining = True

        #endregion-------------REFEREE_SYSTEM_PARAMETERS_INIT-------------- #
        self.shootcallback_count = 0
        #endregion-------BASE_INIT--------#


        #region-----------SUBSCRIBER_NODES_INIT-----------#
        rospy.Subscriber('robot_pose', Odometry, self.getSelfPoseCallback, queue_size=1)
        rospy.Subscriber('gimbal_pose', Quaternion, self.getSelfGimbalCallback, queue_size=1)
        rospy.Subscriber('armor_detection_info', ArmorDetection, self.getArmorDetectionCallback, queue_size=1)
        rospy.Subscriber('field_bonus_status', BonusStatus, self.getBonusStatusCallback, queue_size=1)
        rospy.Subscriber('field_supplier_status', SupplierStatus, self.getSupplierStatusCallback, queue_size=1)
        rospy.Subscriber('game_result', GameResult, self.getGameResultCallback, queue_size=1)
        rospy.Subscriber('game_status', GameStatus, self.getGameStatusCallback, queue_size=1)
        rospy.Subscriber('game_survivor', GameSurvivor, self.getGameSurvivorCallback, queue_size=1)
        rospy.Subscriber('robot_bonus', RobotBonus, self.getRobotBonusCallback, queue_size=1)
        rospy.Subscriber('robot_damage', RobotDamage, self.getRobotDamageCallback, queue_size=1)
        rospy.Subscriber('robot_heat', RobotHeat, self.getRobotHeatCallback, queue_size=1)
        rospy.Subscriber('robot_shoot', RobotShoot, self.getRobotShootCallback, queue_size=1)
        rospy.Subscriber('robot_status', RobotStatus, self.getRobotStatusCallback, queue_size=1)
        rospy.Subscriber('projectile_info', ProjectileInfo, self.SupplyJudge, queue_size=1)
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.TagRPtoPitch, queue_size=1)
        rospy.Subscriber('heat_control_info', HeatControl, self.getProjectileAvailableNumCallback, queue_size=1)
        rospy.Subscriber('rev_from_teammate', TeammateMsg, self.getTeammateMsgCallback, queue_size=1)
        rospy.Subscriber('chassis_gimbal_mode', Debug, self.DebugCallback, queue_size=1)
        rospy.Subscriber('supply_distance', SupplyDistance, self.DeepthBehindCallback, queue_size=1)
        #endregion-----------SUBSCRIBER_NODES_INIT-----------#  

    def DeepthBehindCallback(self, data):
        self.deepth_behind = data.supply_distance

    #region---tag_detection callback------------------------------# 
    def TagRPtoPitch(self,data):
        #rospy.loginfo(data.detections)
        if len(data.detections) != 0:
            tag_pose = data.detections[0].pose.pose.pose
            distance = math.sqrt(math.pow(tag_pose.position.x,2)+math.pow(tag_pose.position.z,2))
            if distance < 4 :
                self.tag_info['tag_detected'] = True
                qx = tag_pose.orientation.x
                qy = tag_pose.orientation.y
                qz = tag_pose.orientation.z
                qw = tag_pose.orientation.w
                angle = euler_from_quaternion((qx,qy,qz,qw))
                self.tag_info['x'] = tag_pose.position.x
                self.tag_info['z'] = tag_pose.position.z
                self.tag_info['pitch'] = math.degrees(angle[1])
                self.tag_info['distance'] = distance
                # rospy.loginfo(self.tag_info)

    #endregion---bulletamount callback--------------------------------#
    
    def SupplyJudge(self, data):
        if self.SURVIVOR_LIST[self.MATE_ID]:
            self.supply_amount = 50
        else:
            self.supply_amount = 100
        
        if self.supply_cooldown == -1: # 冷却结束？
            if self.pjt_info != data.pjt_info:
                self.pjt_info = data.pjt_info
            # if no or little projectile
            if self.pjt_info == 1 or self.pjt_info == 2:
                # if supplier status is close
                if self.SUPPLIER_STATUS == 0:

                    #FIXME:---fix----2019.05.22
                    # if there is projectile and teammate is not supplying
                    if self.SELF_SUPPLY == True:
                    #-----fix------

                        # robot is in the valid supply time
                        if self.MATE_SUPPLY == 0:
                            # go to supply
                            self.supply_needed = True    
            else:
                # if few projectile
                self.supply_needed = False
        # if want to supply again less than 8 seconds
        elif self.supply_cooldown - self.REMAINING_TIME < 8 :
            self.supply_needed = False
        # if want to supply again more than 8 seconds
        else:
            # update the projectile info
            self.pjt_info = data.pjt_info
            # if no or few projectile
            if self.pjt_info == 1 or self.pjt_info == 2:
                if self.SUPPLIER_STATUS == 0:

                    #FIXME:----fix----2019.05.22
                    if self.SELF_SUPPLY == True:
                    #------fix-----

                        if self.MATE_SUPPLY == 0:
                            self.supply_needed = True  
            else:
                self.supply_needed = False

    def BuffJudge(self):
        # print('field is {}, num_dec is {}, self is {}'.format(self.SELF_BONUS_STATUS, self.num_detected_enemy, self.ROBOT_BONUS))
        # print('send_goal_count{}',format(self.gotobuff_count))
        if self.buff_countflag == True and self.gotobuff_count >= 1:
            self.buff_counttime = self.REMAINING_TIME
            self.buff_countflag = False
        if (self.buff_counttime - self.REMAINING_TIME > 15) or (self.REMAINING_TIME % 60 < 12):
            self.buff_countflag = True
            self.gotobuff_count = 0
        # if self.gotobuff_count < 2:
        if self.SELF_BONUS_STATUS != 2 and self.gotobuff_count < 1:
            if self.num_detected_enemy < self.BUFF_detect_num:
                self.buff_needed = True
            

        else:
            # print ('gotobuff_count is {}'.format(self.gotobuff_count))
            self.buff_needed = False


    #region------------BASE_INFO_INIT--------------#

    def normalizeTheta(self, theta): # 将角度degree归一化为[-pi, pi]
        return math.degrees(math.atan2(math.sin(math.radians(theta)), math.cos(math.radians(theta))))

    def getProjectileAvailableNumCallback(self, data): # 100Hz update
        self.ammo_available_num = data.shoot_num
        self.shootcallback_count += 1
        if self.shootcallback_count % self.shoot_ammo_num == 0:
            self.shootSet()
     #       self.shootcallback_count = 0

    def shootSet(self):
        if self.GAME_STATUS == 4:
            if self.ammo_available_num > 0:
                if self.detection_result_front_stable and self.enemy_pose.enemy_dist > 0 and self.enemy_pose.enemy_dist <= 2:
                    self.shoot_cmd.number = min(self.ammo_available_num, 5)
                    self.shoot_cmd.mode = 1
                    self.shoot_mode = 1
                    self.ctrl.shoot(self.shoot_cmd)
                elif self.detection_result_front_stable and self.enemy_pose.enemy_dist > 2 and self.enemy_pose.enemy_dist <= 3:
                    self.shoot_cmd.number = min(self.ammo_available_num, 3)
                    self.shoot_cmd.mode = 1
                    self.shoot_mode = 1
                    self.ctrl.shoot(self.shoot_cmd)
                elif self.num_detected_enemy_front > 5 and self.enemy_pose.enemy_dist > 0 and self.enemy_pose.enemy_dist <= 4:
                    self.shoot_cmd.number = min(self.ammo_available_num, 1)
                    self.shoot_cmd.mode = 1
                    self.shoot_mode = 1
                    self.ctrl.shoot(self.shoot_cmd)
                elif self.shoot_mode != 0:
                    self.shoot_cmd.number = 0
                    self.shoot_cmd.mode = 0
                    self.shoot_mode = 0
                    self.ctrl.shoot(self.shoot_cmd)
            elif self.shoot_mode != 0:
                self.shoot_cmd.number = 0
                self.shoot_cmd.mode = 0
                self.shoot_mode = 0
                self.ctrl.shoot(self.shoot_cmd)
        else:
            pass

    def getTeammateMsgCallback(self, data):
        #std_msgs/Header header
        # uint16 robot_id
        # float64 robot_pos_x
        # float64 robot_pos_y
        # uint8 robot_pjt_info
        # bool find_enemy1

        # bool find_enemy2
        # float64 enemy2_pos_x
        # float64 enemy2_pos_y


        # float64 teammate_pos_x
        # float64 teammate_pos_y
        # uint16 teammate_id
        # uint16 teammate_remain_hp
        # uint8 teammate_pjt_info
        # uint8 cmd
        # float64 enemy1_pos_x
        # float64 enemy1_pos_y

        self.MATE_ID = data.teammate_id
        self.MATE_POSE['x'] = data.teammate_pos_x
        self.MATE_POSE['y'] = data.teammate_pos_y
        self.MATE_REMAIN_HP = data.teammate_remain_hp
        self.MATE_PJT_INFO = data.teammate_pjt_info
        # self.CMD_FROM_MATE = data.cmd
        self.MATE_SUPPLY = data.cmd
        self.ENEMY_POSE_FROM_MATE['x'] = data.enemy1_pos_x
        self.ENEMY_POSE_FROM_MATE['y'] = data.enemy1_pos_y
        # print ('\nenemy_pose_from_mate X :{:.3f}'.format(self.ENEMY_POSE_FROM_MATE['x']))
        # print ('enemy_pose_from_mate Y : {:.3f}'.format(self.ENEMY_POSE_FROM_MATE['y']))

        print ('pub_self_msg')
        # self.team_msg.robot_id = self.SELF_ID
        self.team_msg.teammate_id = self.MATE_ID # 指发送队友的ID
        self.team_msg.robot_pos_x = self.robot_pose['x']
        self.team_msg.robot_pos_y = self.robot_pose['y']
        self.team_msg.robot_pjt_info = self.pjt_info

        self.team_msg.find_enemy1 = self.detection_result
        self.team_msg.enemy1_pos_x = self.ENEMY_POSE_TO_MATE['x']
        self.team_msg.enemy1_pos_y = self.ENEMY_POSE_TO_MATE['y']

        # self.team_msg.mate_supply = self.IS_SUPPLY
        self.team_msg.cmd = self.IS_SUPPLY
        self.team_msg_pub.publish(self.team_msg)
    
    def getSelfPoseCallback(self, data):
        self.robot_pose['x'] = data.pose.pose.position.x
        self.robot_pose['y'] = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        angle = euler_from_quaternion((qx,qy,qz,qw))
        self.robot_pose['theta'] = math.degrees(angle[2])
        # self.is_blocked = self.isBlocked()
        self.isBlocked()
        self.BuffJudge()
        self.point_pub.publish(self.point_pub_)
        # print (self.is_blocked)


    def getSelfGimbalCallback(self, data):
        rpy = euler_from_quaternion([data.x, data.y, data.z, data.w])
        self.gimbal_angle_yaw =  rpy[2]
        #self.gimbal_angle_pitch = rpy[1]

    # gimbal-chassis合法组合: 2-1:摆尾， 1-0:巡逻, 0-0:锁定
    # |chassis\gimbal|   0   |   1   |    2    |
    # |    0-正常     |  锁定  | 巡逻  |  error  |
    # |    1-摆尾     | error | error |   摆尾  |
    # |    2-补弹     | error | error | 补弹摆尾 |
    def GimbalChassisSave(self): # error: chassis_mode-1, gimbal-0
        if self.current_chassis_mode == 1:
            if self.current_gimbal_mode != 2:
                print ('error when in dodge, set enter dodge')
                self.ctrl.gimbal_mode_switch(2)
        elif self.current_chassis_mode == 2:
            if self.current_gimbal_mode != 2:
                print ('error when in supply dodge, set enter supply dodge')
                self.ctrl.gimbal_mode_switch(2)
        elif self.current_chassis_mode == 0:
            if self.current_gimbal_mode == 2:
                print ('error when in normal, set robot patrol')
                self.ctrl.gimbal_mode_switch(1)

    def GimbalChassisJudge(self):
        if self.current_chassis_mode == 0 and self.current_gimbal_mode == 0:
            self.behavior_mode = 0 # 检测锁定模式
        elif self.current_chassis_mode == 0 and self.current_gimbal_mode == 1:
            self.behavior_mode = 1 # 正常巡逻模式
        elif self.current_chassis_mode == 1 and self.current_gimbal_mode == 2:
            self.behavior_mode = 2 # 打击摆尾模式
        elif self.current_chassis_mode == 2 and self.current_gimbal_mode == 2:
            self.behavior_mode = 3 # 补弹摆尾模式
        else:
            self.behavior_mode = -1

    def getArmorDetectionCallback(self, data):# 更新敌人有效坐标位置
        del self.robot_last[:]
        self.robot_last.append([self.robot_pose['x'], self.robot_pose['y']])
        self.detected_by_front = data.detected_by_front
        if data.distance > 0:
            self.enemy_pose.enemy_yaw = math.radians(self.normalizeTheta(math.degrees(data.yaw_angle) + self.robot_pose['theta'])) # radian
            self.enemy_yaw_chassis = data.yaw_angle
            self.enemy_position['x'] = data.distance / 1000.0 * math.cos(self.enemy_pose.enemy_yaw) + self.robot_pose['x']
            self.enemy_position['y'] = data.distance / 1000.0 * math.sin(self.enemy_pose.enemy_yaw) + self.robot_pose['y']
           
            if len(self.enemy_last) < self.detect_length:
                self.enemy_last.append([self.enemy_position['x'], self.enemy_position['y']])
            else:
                self.enemy_last.pop(0)
                self.enemy_last.append([self.enemy_position['x'], self.enemy_position['y']])

            self.detection_result = True
            self.num_detected_enemy += 1
            self.num_not_detected_enemy = 0
            
            self.enemy_pose.enemy_dist = data.distance / 1000.0

            #----fix to test----2019.05.19
            # if self.detected_by_front == True and self.behavior_mode == 1: # 在正常巡逻模式中切换到检测锁定模式
            #     self.ctrl.gimbal_mode_switch(0) # 给视觉锁定
            #     self.ctrl.chassis_mode_switch(0)
            #----fix to test
           
            self.point_pub_.point.x, self.point_pub_.point.y = self.enemy_position['x'], self.enemy_position['y']

            self.ENEMY_POSE_TO_MATE['x'], self.ENEMY_POSE_TO_MATE['y'] = self.enemy_position['x'], self.enemy_position['y']
        else:
            del self.enemy_last[:]
            self.detection_result_stable = False
            self.num_not_detected_enemy += 1
            self.num_detected_enemy = 0
            self.enemy_pose.enemy_dist = 0
            self.point_pub_.point.x, self.point_pub_.point.y = 0, 0

            self.ENEMY_POSE_TO_MATE['x'], self.ENEMY_POSE_TO_MATE['y'] = 0, 0
        
        if self.detected_by_front == False:
            self.num_not_detected_enemy_front += 1
            self.num_detected_enemy_front = 0
        else:
            self.num_not_detected_enemy_front = 0
            self.num_detected_enemy_front += 1 

        if self.num_not_detected_enemy_front > 30:
            self.detection_result_front = False
        else:
            self.detection_result_front = True

        if self.num_detected_enemy_front > 20:
            self.detection_result_front_stable = True
        else:
            self.detection_result_front_stable = False

        if self.num_not_detected_enemy > 50:
            self.detection_result = False
            self.enemy_pose.enemy_dist = 0

        #----fix to test----2019.05.19
        # if self.num_not_detected_enemy > 10 and self.behavior_mode == 0: # 在检测锁定模式中切换到正常巡逻模式
        #     self.ctrl.gimbal_mode_switch(1) # 底盘控制左右摆
        #     self.ctrl.chassis_mode_switch(0)
        #----fix to test
       
        if self.num_detected_enemy > self.detection_result_stable_num: # 稳定检测
            self.detection_result_stable = True

    def isBlocked(self, safe_dist = 0.5, block_dist = 0.3, rate_1 = 4, rate_2 = 40):
        dist_min = float('inf')
        for p in self.obstacle_conner:
            dist = math.hypot(p[0] - self.robot_pose['x'], p[1] - self.robot_pose['y'])
            if dist < dist_min:
                dist_min = dist
                self.block_obstacle = p

        if dist_min > safe_dist: 
            self.block_count = 0
            self.pose_x[:], self.pose_y[:] = [], []
            self.delta_x, self.delta_y = 0, 0
            self.is_blocked = False
            # return False
        else:
            self.block_count += 1
            if self.block_count % rate_1 == 0: # pose update rate: 40
                self.pose_x.append(self.robot_pose['x'])
                self.pose_y.append(self.robot_pose['y'])
            if self.block_count % rate_2 == 0:
                for x in self.pose_x:
                    self.delta_x += abs(x - np.mean(self.pose_x))
                for y in self.pose_y:
                    self.delta_y += abs(y - np.mean(self.pose_y))

                mean_delta_x = self.delta_x / len(self.pose_x)
                mean_delta_y = self.delta_y / len(self.pose_y)
                if mean_delta_x < block_dist and mean_delta_y < block_dist:
                    rospy.logwarn('robot is blocked near point:{}!'.format(self.block_obstacle))
                    self.is_blocked = True
            
    def getEscaspeDirection(self, ratio = 0.5): # 离障碍物太近会堵住无法移动，需要主动摆脱
        # get distance from the closest corner point to the line robot in
        cos_theta, sin_theta = math.cos(math.radians(self.robot_pose['theta'])), math.sin(math.radians(self.robot_pose['theta']))
        line_dist = abs(cos_theta * (self.block_obstacle[1]-self.robot_pose['y']) - sin_theta * (self.block_obstacle[0]-self.robot_pose['x']))
        # to make sure direction
        A = np.mat([[sin_theta, -cos_theta], [-cos_theta, -sin_theta]])
        B = np.mat([[sin_theta * self.robot_pose['x'] - cos_theta * self.robot_pose['y']], [-cos_theta * self.block_obstacle[0] - sin_theta * self.block_obstacle[1]]])
        jion_point = np.linalg.solve(A, B)
        delta_x = jion_point[0] - self.block_obstacle[0]
        delta_y = jion_point[1] - self.block_obstacle[1]
        delta = math.hypot(delta_x, delta_y)

        speed_x, speed_y = ratio * delta_x / delta, ratio * delta_y / delta
        # # escape direction
        # self.block_delta_x = abs(line_dist * sin_theta) * abs(delta_x) / delta_x
        # self.block_delta_y = abs(line_dist * cos_theta) * abs(delta_y) / delta_y
        
        
        # # escape speed in map coordinates
        # speed_x, speed_y = ratio * self.block_delta_x / abs(self.block_delta_x), ratio * self.block_delta_y / abs(self.block_delta_x)
        # escape speed in robot coordinates
        self.escape_v_x = speed_x * cos_theta + speed_y * sin_theta
        self.escape_v_y = - speed_x * sin_theta + speed_y * cos_theta

    # move when less than one meter distance ----NOT-USE
    def TwistMove(self, goal_x, goal_y, error = 0.1, K = 1, d1 = 0.7, K1 = 0.6, d2 = 0.3, K2 = 0.3):
        while True:
            while self.is_blocked:
                self.getEscaspeDirection()
                self.ctrl.send_vel(TwistControl(self.escape_v_x, self.escape_v_y, 0, 0).Twist)
            delta_x = goal_x - self.robot_pose['x'] # 偏移误差, goal_x和goal_y都是地图上的绝对位置
            delta_y = goal_y - self.robot_pose['y']
            delta_dist = math.hypot(delta_x, delta_y) 
            if abs(delta_dist) < error:
                break
            if abs(delta_dist) < d1:
                K = K1
            if abs(delta_dist) < d2:
                K = K2
            cos_theta, sin_theta = math.cos(math.radians(self.robot_pose['theta'])), math.sin(math.radians(self.robot_pose['theta']))
            v_x = K * (cos_theta * delta_x + sin_theta * delta_y)
            v_y = K * (-sin_theta * delta_x + cos_theta * delta_y)
            self.ctrl.send_vel(TwistControl(v_x, v_y, 0, 0).Twist)
        self.ctrl.send_vel(TwistControl(0, 0, 0, 0).Twist)

    def TwitsRotation(self, goal_theta, error_theta = 5, K = 2, theta1 = 90, K1 = 3, theta2 = 60, K2 = 5, theta3 = 20, K3 = 10): # 单位是度
        goal_theta = self.normalizeTheta(goal_theta)
        while True:
            while self.is_blocked:
                self.getEscaspeDirection()
                self.ctrl.send_vel(TwistControl(self.escape_v_x, self.escape_v_y, 0, 0).Twist)
            delta_theta = self.normalizeTheta(goal_theta - self.robot_pose['theta']) # 角度误差
            if abs(delta_theta) < error_theta:
                break
            if abs(delta_theta) < theta1:
                K = K1
            if abs(delta_theta) < theta2:
                K = K2
            if abs(delta_theta) < theta3:
                K = K3
            v_z = K * math.radians(delta_theta)
            self.ctrl.send_vel(TwistControl(0, 0, 0, v_z).Twist)
        self.ctrl.send_vel(TwistControl(0, 0, 0, 0).Twist)

    def getEnemyDirection(self):
        if self.enemy_pose.enemy_dist > 0:
            self.twist_goal_theta = math.degrees(math.atan2(self.enemy_position['y'] - self.robot_pose['y'], self.enemy_position['x'] - self.robot_pose['x']))
        elif self.ARMOR_HIT_NUM == 0: # forward
            self.twist_goal_theta = self.normalizeTheta(self.robot_pose['theta'] + 30)
        elif self.ARMOR_HIT_NUM == 1: # left
            self.twist_goal_theta = self.normalizeTheta(self.robot_pose['theta'] + 90)
        elif self.ARMOR_HIT_NUM == 2: # backward
            self.twist_goal_theta = self.normalizeTheta(self.robot_pose['theta'] + 180)
        elif self.ARMOR_HIT_NUM == 3: # right
            self.twist_goal_theta = self.normalizeTheta(self.robot_pose['theta'] - 90)

    #------NOT USE------#
    def pointToEnemy(self): # move, rotation, 
        toImage = 200
        offset_image, offset = int(0.375 * toImage), 0.2 # math.hypot(0.6, 0.45)/2 = 0.375
        px = int(np.around(self.robot_pose['x'] * toImage)) # 自身转换到图像坐标
        py = 1000 - int(np.around(self.robot_pose['y'] * toImage))

        if px > offset_image and self.map[py, px - offset_image] != 255: # 左边障碍
            self.twits_goal_x = self.robot_pose['x'] + offset
        elif px < (1600 - offset_image) and self.map[py, px + offset_image] != 255: # 右边障碍
            self.twits_goal_x = self.robot_pose['x'] - offset
        elif py > offset_image and self.map[py - offset_image, px] != 255: # 上方障碍
            self.twits_goal_y = self.robot_pose['y'] - offset
        elif py < (1000 - offset_image) and self.map[py + offset_image, px] != 255: # 下方障碍
            self.twits_goal_y = self.robot_pose['y'] + offset
    #------NOT USE------#
        
    def isActionAvaliable(self, goal_x, goal_y, goal_theta, SIZE = 50): # theta是角度  
        # 转换为图像坐标,icra2019_s.pgm像素大小1600*1000,对应场地8*5
        px = int(np.around(goal_x * 200))
        py = 1000 - int(np.around(goal_y * 200))
        quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(goal_theta))
        try:
            # robot_size [0.6 0.45 0.460],占用像素大小[120, 90, 92]
            if (py <= SIZE) or (py >= 1000 - SIZE) or (px <= SIZE) or (px >= 1600 - SIZE):
                rospy.logwarn('isactionavalibe---------goal out map')
                return False
            if (px >= 700) and (px <= 950) and py >= 750:
                rospy.logwarn('isactionavalibe---------goal in enemy supply area')
                return False
            if self.map[py + SIZE, px + SIZE] != 255 or self.map[py - SIZE, px - SIZE] != 255:
                rospy.logwarn('isactionavalibe---------goal near obstacle-1')
                return False
            if self.map[py+SIZE, px - SIZE] != 255 or self.map[py - SIZE, px + SIZE] != 255:
                rospy.logwarn('isactionavalibe---------goal near obstacle-2')
                return False
            self.navgoal.goal.pose.position.x = goal_x
            self.navgoal.goal.pose.position.y = goal_y
            self.navgoal.goal.pose.orientation.x = quat[0]
            self.navgoal.goal.pose.orientation.y = quat[1]
            self.navgoal.goal.pose.orientation.z = quat[2]
            self.navgoal.goal.pose.orientation.w = quat[3]
            self.navgoal.goal.header.stamp = rospy.Time().now()
            return True
        except:
            pass

    def isNear(self, x, y, near_dist = 0.1):
        offset = int(near_dist * 200)
        x_i = int(np.around(x * 200))
        y_i = 1000 -int(np.around(y * 200))

        if y_i - offset <= 0 or y_i + offset >= 1000:
            return True
        if x_i - offset <= 0 or x_i + offset >= 1600:
            return True
        # 以（x, y）为中心的正方形，检查4条边是否有穿过障碍物或超过地图，有则离障碍很近，返回True
        for f in range(-offset, offset, 1):
            if self.map[y_i - offset, x_i + f] < 255:
                return True
            if self.map[y_i + offset, x_i + f] < 255:
                return True
            if self.map[y_i + f, x_i - offset] < 255:
                return True
            if self.map[y_i + f, x_i + offset] < 255:
                return True
        return False

    def isInMap(self, x, y):
        x_i = int(np.around(x * 200))
        y_i = 1000 - int(np.around(y * 200))

        if x_i <= 0 or x_i >= 1600:
            return False
        if y_i <= 0 or y_i >= 1000:
            return False
        if self.map[y_i, x_i] < 255:
            return False
            
        return True
    
    def isReachable(self, enemy_x, enemy_y, self_x, self_y):
        # 转化到图像坐标系
        enemy_x_i = int(np.around(enemy_x * 200))
        enemy_y_i = int(np.around(enemy_y * 200))
        self_x_i = int(np.around(self_x * 200))
        self_y_i = int(np.around(self_y * 200))
        if enemy_x_i >= 1600 or enemy_x_i <= 0 or enemy_y_i >= 1000 or enemy_y_i <= 0:
            return False
        if self_x_i >= 1600 or self_x_i <= 0 or self_y_i >= 1000 or self_y_i <= 0:
            return False

        A = np.mat([[self_x_i, 1], [enemy_x_i, 1]])
        B = np.mat([[self_y_i], [enemy_y_i]])

        if np.linalg.det(A) != 0:
            _k_b_ = np.linalg.solve(A, B)
            if self_x_i > enemy_x_i:
                self_x_i, enemy_x_i = enemy_x_i, self_x_i
            for x in range(self_x_i, enemy_x_i):
                y = int(np.around(_k_b_[0, 0] * x + _k_b_[1, 0]))
                if self.map[1000 - y, x] < 255:
                    return False
        elif self_y_i > enemy_y_i:
            self_x_i, enemy_x_i = enemy_x_i, self_x_i
            for y in range(self_y_i, enemy_y_i):
                if self.map[1000 - y, self_x_i] < 255:
                    return False
        return True

    def enemy_close(self):
        if len(self.enemy_last) < self.detect_length:
            return 0
        elif ((self.enemy_last[9][0] - self.enemy_last[0][0]) * (self.robot_last[0][0] - self.enemy_last[0][0]) + (
            self.enemy_last[9][1] - self.enemy_last[0][1]) * (self.robot_last[0][1] - self.enemy_last[0][1])) > 0:
            if math.hypot(self.enemy_last[9][0] - self.enemy_last[0][0], self.enemy_last[9][1] - self.enemy_last[0][1]) > 0.2:
                return 1 # 靠近
            else:
                return 2 # 远离
    #endregion------------BASE_INFO_INIT--------------#

    #region-----------MATE INIT--------------#

    #endregion-----------MATE INIT------------#

    #region---------------------SEARCH_send_goal----------------------- # 
    def send_goal_in_search(self, goal):
        if self.breakInSearch():
            self.global_path_planner_action_client.cancel_all_goals()
            return False

        # rospy.loginfo('send navigation goal')
        self.global_path_planner_action_client.send_goal(
                goal, feedback_cb=self.global_planner_feedback)
        self.global_path_planner_action_client.wait_for_result(rospy.Duration(6))
        return True
    
    def global_planner_feedback(self, feedback):
        if self.breakInSearch():
            self.global_path_planner_action_client.cancel_all_goals()
            self.local_path_planner_action_client.cancel_all_goals()
            return False

        if len(feedback.path.poses) != 0:
            self.local_path_planner_goal.route = feedback.path
            self.local_path_planner_action_client.send_goal(self.local_path_planner_goal,feedback_cb=self.local_path_planner_feedback_cb)
            return True

    def breakInSearch(self): # break in search 
        if self.is_blocked: # block out
            rospy.logwarn('SEARCH-----get block break-----')
            self.SEARCH_cancel_flag = 1
            return True
        if self.supply_needed: # supply out
            rospy.logwarn('SEARCH-----supply needed break-----')
            self.SEARCH_cancel_flag = 2
            return True

        if self.detection_result:
            if self.enemy_pose.enemy_dist > 0 and self.enemy_pose.enemy_dist < self.SEARCH_stop_dist: # close out
                rospy.logwarn('SEARCH-------stop for {:.3f} distance break-------'.format(self.enemy_pose.enemy_dist))
                self.SEARCH_cancel_flag = 3
                return True
            dist_enemy = math.hypot(self.enemy_position['x']-self.navgoal.goal.pose.position.x, self.enemy_position['y']-self.navgoal.goal.pose.position.y)
            if dist_enemy > 1: # refresh enemy goal if enemy move far away goal point
                rospy.logwarn('SEARCH-------refresh enemy point break--------')
                self.SEARCH_cancel_flag = 4
                return True
        elif self.ARMOR_HIT_NUM != -1: # not detect and hit out
            rospy.logwarn('SEARCH--------robot hit break---------')
            self.SEARCH_cancel_flag = 5
            return True

        qx, qy = self.navgoal.goal.pose.orientation.x, self.navgoal.goal.pose.orientation.y
        qz, qw = self.navgoal.goal.pose.orientation.z, self.navgoal.goal.pose.orientation.w
        angle = euler_from_quaternion((qx, qy, qz, qw))
        goal_x, goal_y, goal_theta = self.navgoal.goal.pose.position.x, self.navgoal.goal.pose.position.y, math.degrees(angle[2])

        if self.ENEMY_BONUS_STATUS == 1:
            if self.detection_result:
                print ('----have detect enemy even ENEMY_BONUS_STAUS is 1----')
            elif [goal_x, goal_y, goal_theta] in self.search_buff_point:
                print ('----have been in the way to search buff----')
            elif self.SEARCH_answer_enemy_buff:
                print ('----have arrived search buff point, but nothing find---')
                # pass
            else:
                rospy.logwarn('SEARCH---------Find enemy getting buff break-----------')
                self.SEARCH_cancel_flag = 6
                return True
        # 先响应队友提供的敌人位置，先转向指向敌人
        # if self.CMD_FROM_MATE == 1: # call for cooperation from mate
        #     rospy.logwarn('SEARCH*********!!!GET CMD TO COOPERATION WITH MATE!!!*********')
        #     self.SEARCH_cancel_flag = 7
        #     return True

        # if self.CMD_FROM_MATE == 2: # call for help from mate
        #     rospy.logwarn('SEARCH*********!!!!!GET CMD TO HELP MATE!!!!!*********')
        #     self.SEARCH_cancel_flag = 8
        #     return True
        if self.ENEMY_POSE_FROM_MATE['x'] != 0 and self.ENEMY_POSE_FROM_MATE['y'] != 0:
            dist_goal_enemy = math.hypot(goal_x - self.ENEMY_POSE_FROM_MATE['x'], goal_y - self.ENEMY_POSE_FROM_MATE['y'])
            if dist_goal_enemy > 1.5:
                rospy.logwarn('SEARCH---------Get enemy position from MATE-----------')
                self.SEARCH_cancel_flag = 7
                return True

        dist_me = math.hypot(self.robot_pose['x']-self.navgoal.goal.pose.position.x, self.robot_pose['y']-self.navgoal.goal.pose.position.y)
        if dist_me < self.SEARCH_cancel_dist:
            self.SEARCH_cancel_flag = 10
            return True
            
        return False

    #endregion---------------------SEARCH_send_goal----------------------- # 

    #region---------------------FOLLOW_send_goal----------------------- # 
    def send_goal_in_follow(self, goal):
        if self.cancelGoalInFollow():
            self.global_path_planner_action_client.cancel_all_goals()            
            rospy.logwarn("-------------cancel_goal_in_follow_send_goal!!!!---------")
            return False
        else:
            self.global_path_planner_action_client.send_goal(goal, feedback_cb=self.global_planner_feedback_follow)
            self.global_path_planner_action_client.wait_for_result(rospy.Duration(2))
            rospy.loginfo('send navigation goal_follow')
            return True

    def global_planner_feedback_follow(self, feedback):
        if self.cancelGoalInFollow():
            self.local_path_planner_action_client.cancel_all_goals()
            self.global_path_planner_action_client.cancel_all_goals()         
            rospy.logwarn("-------------cancel_goal_in_global_path_planner_follow!!!!---------")
            return False
        if len(feedback.path.poses) != 0:
            self.local_path_planner_goal.route = feedback.path
            self.local_path_planner_action_client.send_goal_and_wait(self.local_path_planner_goal)
            # self.local_path_planner_action_client.wait_for_result(rospy.Duration(2))
            return True                   
    
    def cancelGoalInFollow(self):
        if self.supply_needed:
            self.FOLLOW_cancel_flag = 1 # flag 1 for out
            return True
        if self.detection_result_front == False:
            self.FOLLOW_cancel_flag = 1
            return True
        if self.detection_result_front == True:
            dist_enemy = math.hypot(self.enemy_position['x']-self.enemy_follow_x, self.enemy_position['y']-self.enemy_follow_y)
            if dist_enemy > 1:
                self.FOLLOW_cancel_flag = 2 # flag 2 for continue
                return True

        # dist_me = math.hypot(self.robot_pose['x']-self.enemy_position['x'], self.robot_pose['y']-self.enemy_position['y'])
        # if dist_me < self.FOLLOW_stop_dist and :# and self.enemy_close() == 1:
        #     self.FOLLOW_cancel_flag = 3  # close cancle out
        #     return True
        
        #if self.enemy_pose.enemy_dist > 0 and self.enemy_pose.enemy_dist < self.FOLLOW_stop_dist and self.enemy_close() == 1:
        #    self.FOLLOW_cancel_flag = 2
        #    return True    
        self.FOLLOW_cancel_flag = 0
        return False
    #endregion---------------------FOLLOW_send_goal----------------------- #

    #region----------send_goal_origin-----------#
    #send navigatioin goal force
    def send_goal_force(self, goal): # 强制路径规划
        self.sendgoalresult = False
        rospy.loginfo('send navigation goal force!')
        self.global_path_planner_action_client.send_goal(goal, feedback_cb=self.global_planner_feedback_force)
        #self.sendgoalresult = self.global_path_planner_action_client.wait_for_result(rospy.Duration(1))
        self.sendgoalresult = self.global_path_planner_action_client.wait_for_result(rospy.Duration(10))
    def global_planner_feedback_force(self, feedback):
        self.local_path_planner_goal.route = feedback.path
        self.local_path_planner_action_client.send_goal(self.local_path_planner_goal,feedback_cb=self.local_path_planner_feedback_cb)
        return True

    #-----fix------2019.05.21-----cancle send supply goal when mate is getting supply
    def send_goal_in_supply(self, goal):
        self.sendgoalresult = False
        if self.breakInSupply():
            self.global_path_planner_action_client.cancel_all_goals()
            self.local_path_planner_action_client.cancel_all_goals()
            return False
        rospy.loginfo('supply------send navigation goal force!')
        self.global_path_planner_action_client.send_goal(goal, feedback_cb=self.global_planner_feedback_supply)
        self.sendgoalresult = self.global_path_planner_action_client.wait_for_result(rospy.Duration(10))
        
    def global_planner_feedback_supply(self, feedback):
        if self.breakInSupply():
            self.global_path_planner_action_client.cancel_all_goals()
            self.local_path_planner_action_client.cancel_all_goals()
            return False
        self.local_path_planner_goal.route = feedback.path
        self.local_path_planner_action_client.send_goal(self.local_path_planner_goal,feedback_cb=self.local_path_planner_feedback_cb)
        return True

    def breakInSupply(self):
        if self.MATE_SUPPLY == 1:
            rospy.loginfo('----mate is getting supply! cancle my goal----')
            self.SUPPLY_cancle_flag = 1
            return True
        return False

    #send_goal_force
    def send_goal_buff_force(self, goal): # buff强制路径规划
        if self.breakInForceBuff():
            self.global_path_planner_action_client.cancel_all_goals()
            self.local_path_planner_action_client.cancel_all_goals()
            return False
        rospy.loginfo('buff------send navigation goal force!')
        self.global_path_planner_action_client.send_goal(goal, feedback_cb=self.global_planner_buff_feedback_force)
        self.global_path_planner_action_client.wait_for_result(rospy.Duration(6))

    def global_planner_buff_feedback_force(self, feedback):
        if self.breakInForceBuff():
            self.global_path_planner_action_client.cancel_all_goals()
            self.local_path_planner_action_client.cancel_all_goals()
            return False
        if len(feedback.path.poses) != 0:
            self.local_path_planner_goal.route = feedback.path
            self.local_path_planner_action_client.send_goal(self.local_path_planner_goal,feedback_cb=self.local_path_planner_feedback_cb)

    def breakInForceBuff(self):
        if self.SELF_BONUS_STATUS != 0:
            if not ((5.75 <= self.robot_pose['x'] <= 6.85) and (1.25 <= self.robot_pose['y'] <= 2.3)):
                self.BUFF_cancle_flag = 1
                return True
        return False

    # send_goal_buff
    def send_goal_buff(self, goal):
        if self.breakInGoBUFF():
            self.global_path_planner_action_client.cancel_all_goals()
            self.local_path_planner_action_client.cancel_all_goals()
            return False
        self.global_path_planner_action_client.send_goal(goal, feedback_cb=self.global_planner_feedback_buff)
        self.global_path_planner_action_client.wait_for_result(rospy.Duration(7))

    def global_planner_feedback_buff(self, feedback):
        if self.breakInGoBUFF():
            self.global_path_planner_action_client.cancel_all_goals()
            self.local_path_planner_action_client.cancel_all_goals()
            return False
        if len(feedback.path.poses) != 0:
            self.local_path_planner_goal.route = feedback.path
            self.local_path_planner_action_client.send_goal(self.local_path_planner_goal,feedback_cb=self.local_path_planner_feedback_cb)
    
    def breakInGoBUFF(self):
        if self.SELF_BONUS_STATUS != 0:
            if not ((5.75 <= self.robot_pose['x'] <= 6.85) and (1.25 <= self.robot_pose['y'] <= 2.3)):
                self.BUFF_cancle_flag = 1
                return True

        if self.num_detected_enemy > self.BUFF_detect_num:
            rospy.logwarn("BUFF-----------detection_result_enemy {}".format(self.num_detected_enemy))
            dist_to_buff = math.hypot(self.robot_pose['x'] - 6.3, self.robot_pose['y'] - 1.75)
            dist_to_enemy = math.hypot(self.robot_pose['x'] - self.enemy_position['x'], self.robot_pose['y'] - self.enemy_position['y'])
            if dist_to_buff > 1 and dist_to_enemy < 2:
                self.BUFF_cancle_flag = 2
                return True
        return False


    # send navigation goal
    def send_goal(self, goal):
        self.global_path_planner_action_client.send_goal(goal, feedback_cb=self.global_path_planner_feedback_cb)
        self.sendgoalresult = self.global_path_planner_action_client.wait_for_result(rospy.Duration(2))
    # global path planner feedback
    def global_path_planner_feedback_cb(self, feedback):
        if len(feedback.path.poses) != 0:
            self.local_path_planner_goal.route = feedback.path
            self.local_path_planner_action_client.send_goal(self.local_path_planner_goal,feedback_cb=self.local_path_planner_feedback_cb)
        else:
            self.global_path_planner_action_client.cancel_all_goals()
            rospy.logwarn("-------------cancel_goal_in_global_path_planner_fb!!!!---------")
                

    def local_path_planner_feedback_cb(self, feedback):
        pass
    #endregion----------send_goal_origin-----------#


    #region------------------REFEREE_SYSTEM_CALLBACK--------------------- #
    def getBonusStatusCallback(self, data):
        # bonus zone status
        # uint8 UNOCCUPIED = 0
        # uint8 BEING_OCCUPIED= 1
        # uint8 OCCUPIED = 2
        # uint8 red_bonus
        # uint8 blue_bonus
        self.RED_BONUS_STATUS = data.red_bonus
        self.BLUE_BONUS_STATUS = data.blue_bonus
        if abs(self.SELF_ID - 3) < 2:
            self.COLOR = 'RED'
            self.SELF_BONUS_STATUS = data.red_bonus
            self.ENEMY_BONUS_STATUS = data.blue_bonus
        else:
            self.COLOR = 'BLUE'
            self.SELF_BONUS_STATUS = data.blue_bonus
            self.ENEMY_BONUS_STATUS = data.red_bonus

        self.BuffJudge()
        # print ('self_bonus_status is {}'.format(self.SELF_BONUS_STATUS))

    def getGameResultCallback(self, data):
        # game result
        # uint8  DRAW=0
        # uint8  RED_WIN=1
        # uint8  BLUE_WIN=2
        # uint8  result
        pass
    
    def getGameStatusCallback(self, data):
        # game status
        # uint8 PRE_MATCH = 0
        # uint8 SETUP = 1
        # uint8 INIT = 2
        # uint8 FIVE_SEC_CD = 3
        # uint8 ROUND = 4
        # uint8 CALCULATION = 5
        # uint8 game_status
        # uint16 remaining_time
        #rospy.loginfo(data.remaining_time)
        self.GAME_STATUS = data.game_status
        self.REMAINING_TIME = data.remaining_time
        if self.REMAINING_TIME % 60 == 0:
            self.supply_count = 0
            rospy.logwarn("one new minute")
        if self.REMAINING_TIME % 2 == 0:
            self.beated_num_two_secs = self.beated_num
            self.beated_num = 0


        #FIXME:-----fix----2019.05.22
        if self.SURVIVOR_LIST[self.MATE_ID]:
            if self.REMAINING_TIME % 60 >= 30: # 一分钟内的前30秒
                if self.robot_name == 0:
                    self.SELF_SUPPLY = True
                else:
                    self.SELF_SUPPLY = False
            else:
                if self.robot_name == 1:
                    self.SELF_SUPPLY = True
                else:
                    self.SELF_SUPPLY = False
        else:
            self.SELF_SUPPLY = True


    def getGameSurvivorCallback(self, data):
        # robot survival
        # bool red3
        # bool red4
        # bool blue3
        # bool blue4
        self.SURVIVOR_LIST[3] = data.red3 
        self.SURVIVOR_LIST[4] = data.red4
        self.SURVIVOR_LIST[13] = data.blue3
        self.SURVIVOR_LIST[14] = data.blue4

    def getRobotBonusCallback(self, data):
        # robot bonus
        # bool bonus
        self.ROBOT_BONUS = data.bonus

    def getRobotDamageCallback(self, data):
        # robot damage
        # uint8 ARMOR = 0
        # uint8 OFFLINE = 1
        # uint8 EXCEED_HEAT = 2
        # uint8 EXCEED_POWER = 3
        # uint8 damage_type
        # uint8 FORWARD = 0
        # uint8 BACKWARD = 1
        # uint8 LEFT = 2
        # uint8 RIGHT = 3
        # uint8 damage_source

        #forward
        if data.damage_source == 0 and data.damage_type == 0:
            self.ARMOR_HIT_NUM = 0
            self.beated_num += 1
            rospy.logwarn('forward armor damage') 
        #backward
        elif data.damage_source == 1 and data.damage_type == 0:
            self.ARMOR_HIT_NUM = 1
            self.beated_num += 1
            rospy.logwarn('backward armor damage') 
        #left
        elif data.damage_source == 2 and data.damage_type == 0:
            self.ARMOR_HIT_NUM = 2
            self.beated_num += 1
            rospy.logwarn('left armor damage') 
        #right    
        elif data.damage_source == 3 and data.damage_type == 0:
            self.ARMOR_HIT_NUM = 3
            self.beated_num += 1
            rospy.logwarn('right armor damage')
        else:
            self.ARMOR_HIT_NUM = -1 

    def getRobotHeatCallback(self, data):
        # robot power and heat data
        # uint16 chassis_volt
        # uint16 chassis_current
        # float64  chassis_power
        # uint16 chassis_power_buffer
        # uint16 shooter_heat
        self.CHASSIS_VOLT = data.chassis_volt
        self.CHASSIS_CURRENT = data.chassis_current
        self.CHASSIS_POWER = data.chassis_power
        self.CHASSIS_POWER_BUFFER = data.chassis_power_buffer # 无数据
        self.SHOOTER_HEAT = data.shooter_heat # 实时冷却后的热量

    def getRobotShootCallback(self, data): # 无数据
        # robot shoot data
        # uint8 frequency
        # float64 speed
        self.SHOOT_FREQUENCY = data.frequency
        self.SHOOT_SPEED = data.speed

    def getRobotStatusCallback(self, data):
        # robot status
        # uint8 id
        # uint8 level
        # uint16 remain_hp
        # uint16 max_hp
        # uint16 heat_cooling_limit
        # uint16 heat_cooling_rate
        # bool gimbal_output
        # bool chassis_output
        # bool shooter_output
        self.ROBOT_STATUS = data 
        self.SELF_ID = data.id
        self.REMAIN_HP = data.remain_hp
        self.MAX_HP = data.max_hp
        self.HEAT_COOLING_LIMIT = data.heat_cooling_limit
        self.HEAT_COOLING_RATE = data.heat_cooling_rate
        self.GIMBAL_OUTPUT = data.gimbal_output
        self.CHASSIS_OUTPUT = data.chassis_output
        self.SHOOTER_OUTPUT = data.shooter_output

        self.MATE_ID = self.ID_MAP[self.SELF_ID]
        
    def getSupplierStatusCallback(self, data):
        # supplier status
        # uint8 CLOSE = 0
        # uint8 PREPARING = 1
        # uint8 SUPPLYING = 2
        # uint8 status
        if self.SUPPLIER_STATUS != data.status:
            self.SUPPLIER_STATUS = data.status
            if self.SUPPLIER_STATUS == 2:  
                self.supply_count = self.supply_count + self.supply_amount
            if self.supply_count == 100:
                rospy.logwarn("supply is used up")

    #endregion------------------REFEREE_SYSTEM_CALLBACK--------------------- #
    
    def DebugCallback(self, data):
        self.current_gimbal_mode = data.gimbal_mode
        self.current_chassis_mode = data.chassis_mode
        self.GimbalChassisSave()
        self.GimbalChassisJudge()

        
if __name__ == '__main__':
    rospy.init_node('Battle')
    env = BattleEnv()
    rospy.spin()
