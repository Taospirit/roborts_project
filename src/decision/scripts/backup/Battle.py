#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
from random import uniform
import numpy as np
from PIL import Image
import rospy
import tf
import actionlib
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from controller import Controller
from geometry_msgs.msg import Twist, PoseStamped
from roborts_msgs.msg import GlobalPlannerGoal
from roborts_msgs.msg import ArmorDetectionAction, ArmorDetectionGoal, ArmorDetection
from roborts_msgs.msg import GlobalPlannerAction, LocalPlannerAction, LocalPlannerGoal, GlobalPlannerFeedback
from roborts_msgs.msg import GimbalAngle, GimbalRate, ShootInfo, ShootState, TwistAccel
from roborts_msgs.srv import ShootCmd, GimbalMode, FricWhl
from roborts_msgs.msg import BonusStatus, GameResult, GameStatus, GameSurvivor, ProjectileSupply, RobotBonus, RobotDamage
from roborts_msgs.msg import RobotHeat, RobotShoot, RobotStatus, SupplierStatus
from decision.msg import EnemyPos
#
from decision.msg import BulletAmount
from apriltags2_ros.msg import AprilTagDetectionArray
# added by JK

class BattleEnv():
    def __init__(self):
        
        self.map = np.array(
            Image.open("{}/icra2019_s.pgm".format(
                os.path.split(os.path.realpath(__file__))[0])))
        self.controller = Controller()
        self.navgoal = GlobalPlannerGoal()
        self.navgoal.goal.header.frame_id = 'map'
        self.gimbal_angle = GimbalAngle()
        self.gimbal_rate = GimbalRate()
        self.shoot_info = ShootInfo()
        self.shoot_state = ShootState()
        self.twist_accel = TwistAccel()
        self.armor_detection_result = ArmorDetection()
        
        # armor detection action
        self.armor_detection_action_client = actionlib.SimpleActionClient(
            'armor_detection_node_action', ArmorDetectionAction)
    	self.armor_detection_action_client.wait_for_server(rospy.Duration(0.5))
        self.armor_detection_goal = ArmorDetectionGoal
        self.enemy_pose = EnemyPos()
        self.enemy_pose.enemy_dist = 0
        self.num_not_detected_enemy = 0
        self.num_no_enemy = 0
        self.num_detected_enemy = 0
        self.tf_pose = PoseStamped()
        self.global_tf_pose = PoseStamped()
        self.tf_tansformer = TransformListener()
        self.quatarnion = Quaternion()
        self.enemy_pose_robot_frame = EnemyPos()
        # self.controller.set_gimbal_mode_client(1)
        self.enemy_pose = EnemyPos()
        
        self.ammo_num = 40 # 弹量
        # self.not_detected_enemy = 0
        # self.num_no_enemy = 0
        # self.enemy_historical_position = []
        self.num_detected_enemy = 0
        self.num_not_detected_enemy = 0
        self.point_to_enemy_radian = 0
        self.STOP_DIST = 1.5

        self.enemyblock_num = 0
        self.enemyblock_attacknum = [[2.3, 3.2, -45], [2.3, 3.2, 45], [1, 3.15, -90]]
        self.enemyblock_num_robot1 = 0
        self.enemyblock_attacknum_robot1 = [[5.7, 1.8, 145], [5.7, 1.8, -145], [7, 1.85, 90]]

        self.detection_result = False
        self.detection_result_stable = False
        self.sudden_find = False
        
        # path planner action to execute the navigation goal
        self.global_path_planner_action_client = actionlib.SimpleActionClient(
            'global_planner_node_action', GlobalPlannerAction)
        self.local_path_planner_action_client = actionlib.SimpleActionClient(
            'local_planner_node_action', LocalPlannerAction)
        self.local_path_planner_goal = LocalPlannerGoal
        
        self.search_regions = list()
        self.search_region_1 = list()
        self.search_region_2 = list()
        self.search_region_3 = list()
        self.search_region_4 = list()
        
        '''
        self.search_region_1.append([2.86, 2.50, -1.57])
        self.search_region_1.append([1.78, 0.94, 3.14])
        self.search_region_1.append([1.2, 2.18, 1.57])
        self.search_region_2.append([5.35, 0.8, 0])
        self.search_region_2.append([7.40, 1.90, 1, 57])
        self.search_region_2.append([5.35, 2.24, 3.14])
        self.search_region_3.append([3.05, 4.56, 3.14])
        self.search_region_3.append([0.8, 3.72, -1.57])
        self.search_region_3.append([2.84, 3.47, 0])
        self.search_region_4.append([5.55, 2.93, 1.57])
        self.search_region_4.append([6.14, 4.34, 0])
        self.search_region_4.append([7.23, 2.55, 1.57])
        '''
        
        self.search_region_1.append([2.86, 2.50, math.degrees(uniform(0,3.14))])
        self.search_region_1.append([1.78, 0.94, math.degrees(uniform(0,3.14))])
        self.search_region_1.append([1.2, 2.18, math.degrees(uniform(0,3.14))])
        self.search_region_2.append([5.35, 0.8, math.degrees(uniform(0,3.14))])
        self.search_region_2.append([7.40, 1.90, math.degrees(uniform(0,3.14))])
        self.search_region_2.append([5.35, 2.24, math.degrees(uniform(0,3.14))])
        self.search_region_3.append([3.05, 4.56, math.degrees(uniform(0,3.14))])
        self.search_region_3.append([0.8, 3.72, math.degrees(uniform(0,3.14))])
        self.search_region_3.append([2.84, 3.47, math.degrees(uniform(0,3.14))])
        self.search_region_4.append([5.55, 2.93, math.degrees(uniform(0,3.14))])
        self.search_region_4.append([6.14, 4.34, math.degrees(uniform(0,3.14))])
        self.search_region_4.append([7.23, 2.55, math.degrees(uniform(0,3.14))])
        self.search_regions.append(self.search_region_1)
        self.search_regions.append(self.search_region_2)
        self.search_regions.append(self.search_region_3)
        self.search_regions.append(self.search_region_4)
        self.last_position = PoseStamped()
        self.search_num = 0

        self.robot_pose = {'x': 1, 'y': 1, 'theta': 0}
        self.robot_pose_1 = {'x': 1.5, 'y': 1, 'theta': 0}
        self.enemy_position = {'x':0, 'y':0, 'theta':0}
	    self.enemy_pose.enemy_dist = float('inf')
	
        self.enemy_pose = EnemyPos()
        # self.enemy_pose_last = EnemyPos()
        self.dist_from_me = []
        self.dist_from_him = []

        self.which_armor = -1
        self.num_no_enemy = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0

        self.goal_x_robot1 = 0
        self.goal_y_robot1 = 0
        self.goal_yaw_robot1 = 0
        self.choosed_goal_x = 0
        self.choosed_goal_y = 0
        self.choosed_goal_yaw = 0
        self.choosed_goal_x_robot1 = 0
        self.choosed_goal_y_robot1 = 0
        self.choosed_goal_yaw_robot1 = 0
        self.far_choosed_goal_x = 0
        self.far_choosed_goal_y = 0
        self.far_choosed_goal_yaw = 0
        self.far_choosed_goal_x_robot1 = 0
        self.far_choosed_goal_y_robot1 = 0
        self.far_choosed_goal_yaw_robot1 = 0

        self.enemyblock_num = 0
        self.enemyblock_attacknum = [[2.3, 3.2, -45], [2.3, 3.2, 45], [1, 3.15, -90]]
        self.enemyblock_num_robot1 = 0
        self.enemyblock_attacknum_robot1 = [[5.7, 1.8, 145], [5.7, 1.8, -145], [7, 1.85, 90]]

        self.dist_from_him = []
        self.dist_from_me = []
        self.dist_from_him_robot1 = []
        self.dist_from_me_robot1 = []
        self.follow_goal_x = 0
        self.follow_goal_y = 0
        self.follow_goal_yaw = 0
        self.follow_goal_x_robot1 = 0
        self.follow_goal_y_robot1 = 0
        self.follow_goal_yaw_robot1 = 0

        self.armor_nonecount = 0
        self.deputy_nonecount = 0
        self.blocked_thetas = []

        self.supply_needed = False
        self.supplypoint_list = [[3.86, 3.6, 90],[3.7, 3.6, 90], [3.8, 1.5, -90], [3.7, 1.5, -90]]
        self.supply_pointnum = 0
        self.sendgoalresult = False
        self.tag_info = {'tag_detected': False, 'x': 0, 'z': 0, 'pitch': 0, 'distance': 0 }
	#added by JK

        rospy.Subscriber('robot_pose', Odometry, self.getSelfPoseCallback,queue_size=1)
        rospy.Subscriber('gimbal_pose', Quaternion, self.getSelfGimbalCallback,queue_size=1)
        rospy.Subscriber('armor_detection_info', ArmorDetection, self.getArmorDetectionCallback, queue_size=1)
        rospy.Subscriber('field_bonus_status', BonusStatus, self.getBonusStatusCallback)
        rospy.Subscriber('field_supplier_status', SupplierStatus, self.getSupplierStatusCallback)
        rospy.Subscriber('game_result', GameResult, self.getGameResultCallback)
        rospy.Subscriber('game_status', GameStatus, self.getGameStatusCallback)
        rospy.Subscriber('game_survivor', GameSurvivor, self.getGameSurvivorCallback)
        rospy.Subscriber('projectile_supply', ProjectileSupply, self.getProjectileSupplyCallback)
        rospy.Subscriber('robot_bonus', RobotBonus, self.getRobotBonusCallback)
        rospy.Subscriber('robot_damage', RobotDamage, self.getHurtInfoCallback)
        rospy.Subscriber('robot_heat', RobotHeat, self.getRobotHeatCallback)
        rospy.Subscriber('robot_shoot', RobotShoot, self.getRobotShootCallback)
        rospy.Subscriber('robot_status', RobotStatus, self.getRobotStatusCallback)
	#rospy.Subscriber('enemy_pose', ArmorDetection, self.getArmorDetectionCallback, queue_size=1) # add when testing simulation
	#
	    rospy.Subscriber('bullet_amount',BulletAmount,self.BulletAmountJudge)
        rospy.Subscriber('tag_detections',AprilTagDetectionArray,self.TagRPtoPitch)
	

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
                rospy.loginfo(self.tag_info)

    def BulletAmountJudge(self,data):
        if data.state == True:
            self.supply_needed = True
            self.supply_pointnum = (data.point_num - 1) * 2
        else:   
            self.supply_needed = False
       
	# added by JK
         
    def getSelfPoseCallback(self, data):
        self.robot_pose['x'] = data.pose.pose.position.x
        self.robot_pose['y'] = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        angle = euler_from_quaternion((qx,qy,qz,qw))
        self.robot_pose['theta'] = math.degrees(angle[2])
        self.blockedPoseAnalysis(self.robot_pose['x'], self.robot_pose['y'],
                                 self.robot_pose['theta'])

    def blockedPoseAnalysis(self, x, y, theta):
        if ((x > 0.8 and x < 1.8 and y > 1.2 and y < 2.5) or
            (x > 2.1 and x < 3.1 and y > 0 and y < 2.5) or
            (x > 4.9 and x < 5.9 and y > 2.5 and y < 5) or
            (x > 6.2 and x < 7.5 and y > 2.5 and y < 3.8)):
            del self.blocked_thetas[:]
            self.blocked_thetas.append(90)
            self.blocked_thetas.append(-90)
        elif ((x > 0 and x < 2.1 and y > 2.7 and y < 3.7) or 
              (x > 0 and x < 2.1 and y > 4 and y < 5) or 
              (x > 5.9 and x < 8 and y > 1.3 and y < 2.3) or
              (x > 5.9 and x < 8 and y > 0 and y < 1)):
            del self.blocked_thetas[:]
            self.blocked_thetas.append(0)
            self.blocked_thetas.append(178)
            self.blocked_thetas.append(-178)
        else:
            del self.blocked_thetas[:]
            self.blocked_thetas.append(0)
            self.blocked_thetas.append(90)
            self.blocked_thetas.append(178)
            self.blocked_thetas.append(-178)
            self.blocked_thetas.append(-90) 

    def getArmorDetectionCallback(self, data):
        self.enemy_pose.enemy_yaw = data.yaw_angle
        # self.enemy_pose.enemy_dist = data.distance / 1000

        if data.distance > 0:
            self.detection_result = True
            self.num_detected_enemy += 1
            self.num_not_detected_enemy = 0

            self.enemy_pose.enemy_dist = data.distance / 1000.0
            self.controller.set_gimbal_mode_client(0) # 测试仿真时注释
        else:
            self.detection_result_stable = False
            self.num_not_detected_enemy += 1
            self.num_detected_enemy = 0

        if self.num_detected_enemy == 1:
            self.sudden_find = True
        else:
            self.sudden_find = False

        if self.num_not_detected_enemy > 10:
            self.detection_result = False
            self.enemy_pose.enemy_dist = 0
            self.controller.set_gimbal_mode_client(1) # 测试仿真时注释

        if self.num_detected_enemy > 10: # 稳定检测
            self.detection_result_stable = True

        #self.getEnemyPose(self.enemy_pose)
        # if data.distance > 0:
        #         self.num_detected_enemy += 1
        #         self.detection_result = True
        # else:
        #         self.num_detected_enemy = 0
        #         self.detection_result = False
        # if not self.detection_result:
        #         self.not_detected_enemy += 1
        #         self.num_detected_enemy = 0
        # else:
        #         self.not_detected_enemy = 0
        #         self.enemy_pose.enemy_dist = data.distance / 1000.0
        #         self.controller.set_gimbal_mode_client(0)
        #         self.controller.set_gimbal_mode_client(0)
        # if self.not_detected_enemy > 30:
        #         self.enemy_pose.enemy_dist = 0
        #         self.detection_result = False
        #         self.controller.set_gimbal_mode_client(1)
        # self.getEnemyPose(self.enemy_pose)         

    def isActionAvaliable(self, goal_x, goal_y, goal_theta): # theta是角度  
        # 转换为图像坐标,icra2019_s.pgm像素大小1600*1000,对应场地8*5
        px = int(np.around(goal_x * 200))
        py = 1000 - int(np.around(goal_y * 200))
        quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(goal_theta))
        try:
            # robot_size [0.6 0.45 0.460],占用像素大小[120, 90, 92]
            SIZE = 50
            if (py <= 60) or (py >= 940) or (px <= 60) or (px >= 1540):
                return False
            if self.map[py+SIZE, px+SIZE] != 255 or self.map[py-SIZE, px-SIZE] != 255:
                return False
            if self.map[py+SIZE, px-SIZE] != 255 or self.map[py-SIZE, px+SIZE] != 255:
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
    

    '''
    def Enemypose_Analysis(self, goal_x, goal_y, goal_yaw):
        if goal_x >= 0 and goal_x <= 1.9 and goal_y >= 0 and goal_y <= 2.35:
            self.enemyblock_num = 1
            # 清空并引入块集
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([1.3, 2.15, -95])
            self.enemyblock_attacknum.append([1.9, 0.8, 170])
            # 清空并引入距离数组
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]

        elif goal_x >=0 and goal_x <= 1.9 and goal_y > 2.35 and goal_y <= 4:
            self.enemyblock_num = 2
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([1.35, 2.4, 100])
            self.enemyblock_attacknum.append([0.65, 4, -75])
            self.enemyblock_attacknum.append([2.35, 3.25, 178])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif goal_x >= 0 and goal_x <= 1.9 and goal_y > 4 and goal_y <= 5:
            self.enemyblock_num = 3
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([0.65, 3.7, 70])
            self.enemyblock_attacknum.append([2.35, 4.5, 178])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif (goal_x > 1.9 and goal_x <= 4.7 and goal_y >= 3 and goal_y <= 5) or (goal_x > 1.9 and goal_x <= 4 and goal_y >= 2 and goal_y <= 3):
            self.enemyblock_num = 4
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([2, 4.5, -55])
            self.enemyblock_attacknum.append([2, 3.2, 0])
            self.enemyblock_attacknum.append([2.65, 2.55, 45])
            self.enemyblock_attacknum.append([4, 2.5, 135])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif goal_x >1.9 and goal_x <= 3.15 and goal_y >= 0 and goal_y <= 2:
            self.enemyblock_num = 5
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([1.1, 0.75, 0])
            self.enemyblock_attacknum.append([2.65, 1.67, -90])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif (goal_x >= 3.15 and goal_x <= 6 and goal_y >= 0 and goal_y <= 2) or (goal_x >= 4 and goal_x <= 6 and goal_y >= 2 and goal_y <= 3):
            self.enemyblock_num = 6
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([4, 2.5, -45])
            self.enemyblock_attacknum.append([5.5, 2.47, -135])
            self.enemyblock_attacknum.append([6, 1.8, -165])
            self.enemyblock_attacknum.append([6.1, 0.5, 135])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 0 and goal_y <= 1:
            self.enemyblock_num = 7
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([5.4, 0.5, 0])
            self.enemyblock_attacknum.append([7.45, 1.8, -90])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 1 and goal_y <= 2.6:
            self.enemyblock_num = 8
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([7.45, 1.1, 120])
            self.enemyblock_attacknum.append([5.5, 1.8, 0])
            self.enemyblock_attacknum.append([6.75, 2.65, -60])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 2.6 and goal_y <= 5:
            self.enemyblock_num = 9
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([6.7, 2.75, 65])
            self.enemyblock_attacknum.append([5.75, 4.3, -30])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        elif goal_x > 4.7 and goal_x <= 6 and goal_y >= 3 and goal_y <= 5:
            self.enemyblock_num = 10
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([5.45, 2.9, 90])
            self.enemyblock_attacknum.append([7, 4.3, -175])
            for k in range(0, len(self.dist_from_me)):
                del self.dist_from_me[0]
            for k in range(0, len(self.dist_from_him)):
                del self.dist_from_him[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.dist_from_him.append(
                    np.square(k[1][0] - self.robot_pose_1['x']) + np.square(k[1][1] - self.robot_pose_1['y']))
                self.dist_from_me.append(np.square(k[1][0] - self.robot_pose['x']) + np.square(k[1][1] - self.robot_pose['y']))
            if self.dist_from_me.index(min(self.dist_from_me)) != self.dist_from_him.index(min(self.dist_from_him)):
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
            else:
                del self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))]
                del self.dist_from_him[self.dist_from_him.index(min(self.dist_from_him))]
                del self.dist_from_me[self.dist_from_me.index(min(self.dist_from_me))]
                self.choosed_goal_x_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][0]
                self.choosed_goal_y_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][1]
                self.choosed_goal_yaw_robot1 = self.enemyblock_attacknum[self.dist_from_him.index(min(self.dist_from_him))][2]
                self.far_choosed_goal_x = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][0]
                self.far_choosed_goal_y = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][1]
                self.far_choosed_goal_yaw = self.enemyblock_attacknum[self.dist_from_me.index(min(self.dist_from_me))][2]
        else:
            print 'enemy_analysis out of map!!!!!!!!!!!1'
    '''
    
    
    '''
    # send detection command
    def send_detection_cmd(self, command):
        rospy.loginfo('send detection command')
        self.armor_detection_goal.command = command
        self.armor_detection_action_client.send_goal(self.armor_detection_goal)

    def armor_detection_feedback_cb(self, feedback):
        if feedback.detected:
            self.not_detected_enemy = 0
            self.controller.set_gimbal_mode_client(0)
            self.controller.set_gimbal_mode_client(0)
            self.controller.set_gimbal_mode_client(0)
            self.controller.set_gimbal_mode_client(0)
            self.controller.set_gimbal_mode_client(0)
            #rospy.loginfo('find enemy')
            camera_pose_msg = feedback.enemy_pos
            distance = math.sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x + \
            camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y)
            yaw = math.atan2(camera_pose_msg.pose.position.y, camera_pose_msg.pose.position.x + 0.0000001)
            #self.quatarnion = quaternion_from_euler(0, 0, yaw)
            #camera_pose_msg.header.frame_id = 'industry_camera'
            #camera_pose_msg.pose.orientation.x = self.quatarnion[0]
            #camera_pose_msg.pose.orientation.y = self.quatarnion[1]
            #camera_pose_msg.pose.orientation.z = self.quatarnion[2]
            #camera_pose_msg.pose.orientation.w = self.quatarnion[3]
            self.enemy_pose.enemy_dist = distance / 1000.0 
            self.enemy_pose.enemy_yaw = yaw 
            
            if self.enemy_pose.enemy_dist == 0:
                self.detection_result = False
            else:
                self.detection_result = True
            #rospy.logwarn('detection_result: {}, distance:{} '.format(self.detection_result,self.enemy_pose.enemy_dist))                      
            self.getEnemyPose(self.enemy_pose)
        else:
            #rospy.logwarn('no enemy')
            self.not_detected_enemy = self.not_detected_enemy + 1
        if self.not_detected_enemy > 10:
            self.not_detected_enemy = 0
            self.enemy_pose.enemy_dist = 0 
            self.enemy_pose.enemy_yaw = 0 
            #self.getEnemyPose(self.enemy_pose)
            self.controller.set_gimbal_mode_client(1)
        '''

    # send navigation goal
    def send_goal(self, goal):
        if not self.sudden_find:
            rospy.loginfo('send navigation goal')
            self.global_path_planner_action_client.send_goal(
                goal, feedback_cb=self.global_path_planner_feedback_cb)
            #self.global_path_planner_action_client.wait_for_result(rospy.Duration(2))
	    self.sendgoalresult = self.global_path_planner_action_client.wait_for_result(rospy.Duration(2))
        else:
             self.global_path_planner_action_client.cancel_all_goals()

    # global path planner feedback
    def global_path_planner_feedback_cb(self, feedback):
        if not self.sudden_find:
            if len(feedback.path.poses) != 0:
                self.local_path_planner_goal.route = feedback.path
                self.local_path_planner_action_client.send_goal(self.local_path_planner_goal,feedback_cb=self.local_path_planner_feedback_cb)
        else:
             self.global_path_planner_action_client.cancel_all_goals()
                
    def local_path_planner_feedback_cb(self, feedback):
        if self.sudden_find:
            self.local_path_planner_action_client.cancel_all_goals()
   
    def getSelfGimbalCallback(self, data):
        rpy = euler_from_quaternion([data.x, data.y, data.z, data.w])
        self.gimbal_angle_yaw =  rpy[2]
        self.gimbal_angle_pitch = rpy[1]

    def getHurtInfoCallback(self, data):
        #forward
        if data.damage_source == 0 and data.damage_type == 0:
            self.which_armor = 0
            self.armor_nonecount = 0
            print 'armor 0'

        #backward
        elif data.damage_source == 1 and data.damage_type == 0:
            self.which_armor = 1
            self.armor_nonecount = 0
            print 'armor 1'
        
        #left
        elif data.damage_source == 2 and data.damage_type == 0:
            self.which_armor = 2
            self.armor_nonecount = 0
            print 'armor 2'

        #right    
        elif data.damage_source == 3 and data.damage_type == 0:
            self.which_armor = 3
            self.armor_nonecount = 0
            print 'armor 3'
        else:
            if self.armor_nonecount == 2:
                self.which_armor = -1
                self.armor_nonecount = 0
                print 'armor -1'
            else:
                self.armor_nonecount = self.armor_nonecount + 1

    def getBonusStatusCallback(self, data):
        self.field_bonus_status = data
        self.field_red_bonus_status = data.red_bonus
        self.field_blue_bonus_status = data.blue_bonus

    def getSupplierStatusCallback(self, data):
        self.field_supplier_status = data.status

    def getGameResultCallback(self, data):
        pass

    def getGameStatusCallback(self, data):
        self.game_status = data 
        self.five_seconds_count = data == 3
        self.remaining_time = data.remaining_time

    def getGameSurvivorCallback(self, data):
        self.red3_survivor = data.red3
        self.red4_survior = data.red4
        self.blue3_survivor = data.blue3
        self.blue4_survivor = data.blue4

    def getProjectileSupplyCallback(self, data):
        self.projectile_supply = data.supply

    def getRobotBonusCallback(self, data):
        self.robot_bonus = data.bonus
    
    def getRobotHeatCallback(self, data):
        self.robot_heat = data
        self.shooter_heat = data.shooter_heat 

    def getRobotShootCallback(self, data):
        self.shoot_frequency = data.frequency
        self.shoot_speed = data.speed
         
    def getRobotStatusCallback(self, data):
        self.robot_status = data 
        self.remain_hp = data.remain_hp
        

if __name__ == '__main__':
    rospy.init_node('Battle')
    env = BattleEnv()
    rospy.spin()
