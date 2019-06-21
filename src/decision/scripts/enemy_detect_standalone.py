#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import random
import math
import rospy
import tf
import numpy as np
from PIL import Image
from decision.msg import EnemyPos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from PIL import Image
from roborts_msgs.msg import ArmorDetectionAction, ArmorDetectionGoal, ArmorDetection


def getSelfPoseCallback(data):
    robot_pose['x'] = data.pose.pose.position.x
    robot_pose['y'] = data.pose.pose.position.y
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    angle = math.atan2(2 * (qx * qy + qz * qw),
                       qw * qw + qx * qx - qy * qy - qz * qz)
    robot_pose['theta'] = math.degrees(angle)

def changeDirection(begin_x, begin_y, end_x, end_y, pose_x, pose_y):
    if begin_x == end_x and abs(pose_y - begin_y) > abs(end_y - begin_y):
        return True
    elif abs(pose_x - begin_x) > abs(end_x - begin_x):
        return True
    else:
        return False

def normalizeTheta(theta): # 将角度归一化为[-180, 180]
    return math.degrees(math.atan2(math.sin(math.radians(theta)), math.cos(math.radians(theta))))

def isInSight(enemy_x, enemy_y, self_x, self_y, self_yaw, visible_dist, visible_theta): # yaw单位是角度
    dist = math.sqrt(math.pow(enemy_x - self_x, 2) + math.pow(enemy_y - self_y, 2))
    theta = 0
    if dist > visible_dist:
        print ('false_dist_out %s' % dist)
        return False, 0, 0
    else:# 角度归一化
        self_theta = normalizeTheta(self_yaw)
        theta = math.atan2(enemy_y - self_y, enemy_x - self_x)
        theta_diff = normalizeTheta(math.degrees(theta) - self_theta)
        if abs(theta_diff) > visible_theta:
            print ('false_theta_out %s' % theta_diff)
            return False, 0, 0
    print ('dist is %s, theta_diff is %s' % (dist, theta_diff))
    return True, dist, theta # 避免后面重复计算

def isReachable(enemy_x, enemy_y, self_x, self_y):
    # 转化到图像坐标系
    enemy_x_i = int(np.around(enemy_x * 200))
    enemy_y_i = int(np.around(enemy_y * 200))
    self_x_i = int(np.around(self_x * 200))
    self_y_i = int(np.around(self_y * 200))

    A = np.mat([[self_x_i, 1], [enemy_x_i, 1]])
    B = np.mat([[self_y_i], [enemy_y_i]])

    if np.linalg.det(A) != 0:
        _k_b_ = np.linalg.solve(A, B)
        if self_x_i > enemy_x_i:
            self_x_i, enemy_x_i = enemy_x_i, self_x_i
        for x in range(self_x_i, enemy_x_i):
            y = int(np.around(_k_b_[0, 0] * x + _k_b_[1, 0]))
            if map[1000 - y, x] < 255:
                print ('false_unreachable_in_line')
                return False
    elif self_y_i > enemy_y_i:
        self_x_i, enemy_x_i = enemy_x_i, self_x_i
        for y in range(self_y_i, enemy_y_i):
            if map[1000 - y, self_x_i] < 255:
                print ('fasle_unreachable_in_y')
                return False
    print ('isreachable')
    return True

if __name__ == '__main__':
    rospy.init_node('enemy_detect')
    rate = rospy.Rate(20)
    robot_pose = {'x': 1, 'y': 1, 'theta': 0}
    rospy.Subscriber('robot_pose', Odometry, getSelfPoseCallback, queue_size=1)

    enemy_point_pub = rospy.Publisher('enemy_point', PointStamped, queue_size=1)
    enemy_point_ = PointStamped()   
    enemy_point_.header.frame_id = "map"
    enemy_point_.header.stamp = rospy.Time()
    enemy_point_.point.z = 0   

    enemy_pose = ArmorDetection()
    enemy_pose_pub = rospy.Publisher(
        'armor_detection_info', ArmorDetection, queue_size=1)

    map = np.array(
        Image.open("{}/icra2019_s.pgm".format(
            os.path.split(os.path.realpath(__file__))[0]))) 

    p_list = [[0.7, 0.7], [0.7, 4.5], [3.5, 4.5], 
                [3.5, 3.3], [5.5, 3.3], [5.5, 4.5], 
                [7.3, 4.5], [7.3, 1.8],[2.5, 1.8], [2.5, 0.7]]
    test_list = [[4, 1.5], [4.1, 1.5], [4.02, 1.55], [4.3, 1.5], [4.5, 1.5], [4.7, 1.5]]
    p_list_2 = [[2.5, 1.5], [5.5, 1.5], [5.5, 3.5], [2.5, 3.5]]
    p_list_3 = [[2.5, 1.5], [7.5, 1.5]]
    enemy_point = {'x': 0, 'y': 0, 'theta': 0}
    
    i = 0
    j = i+1
    STEP = 0.06 # 敌人移动步长
    V_DIST = 10 # 视觉检测有效距离
    V_THETA = 30 # 视觉检测有效左右角度
    enemy_point['x'], enemy_point['y'] = p_list[i][0], p_list[i][1]
    count = 0

    while not rospy.is_shutdown():
        if p_list[i][0] == p_list[j][0]:
            if p_list[i][1] < p_list[j][1]:
                enemy_point['y'] += STEP
            else:
                enemy_point['y'] -= STEP
        elif p_list[i][0] < p_list[j][0]:
            enemy_point['x'] += STEP
        else:
            enemy_point['x'] -= STEP
        
        if changeDirection(p_list[i][0], p_list[i][1], p_list[j][0], p_list[j][1], enemy_point['x'], enemy_point['y']):
            i += 1
            j = i +1 
            i %= len(p_list)
            j %= len(p_list)
            enemy_point['x'], enemy_point['y'] = p_list[i][0], p_list[i][1]
        
        count = int(count + 1) % 6
        enemy_point_.point.x = enemy_point['x']
        enemy_point_.point.y = enemy_point['y']
        # enemy_point_.point.x = 5 # 静态测试用点
        # enemy_point_.point.y = 3
        # enemy_point_.point.x = test_list[count][0]
        # enemy_point_.point.y = test_list[count][1]
        enemy_point_pub.publish(enemy_point_)

        _isInSight, _dist, _theta = isInSight(enemy_point_.point.x, enemy_point_.point.y, robot_pose['x'], robot_pose['y'], robot_pose['theta'], V_DIST, V_THETA)
        _isReachable = isReachable(enemy_point_.point.x, enemy_point_.point.y, robot_pose['x'], robot_pose['y'])

        if _isInSight and _isReachable:
            enemy_pose.yaw_angle = _theta - math.radians(robot_pose['theta'])
            enemy_pose.distance = _dist * 1000
            enemy_pose.detected_enemy = 1
            enemy_pose_pub.publish(enemy_pose)
            rospy.logwarn("enemy in sight!")
            print ('self pose: %s\t%s\t' % (robot_pose['x'], robot_pose['y']))
            print ('enemy pose: %s\t%s\t\n' % (enemy_point_.point.x, enemy_point_.point.y))
            # print ('enemy_pose: %s\t%s\t\n' % (robot_pose['x']+_dist*math.cos(_theta), robot_pose['y']+_dist*math.sin(_theta)))
        else:
            enemy_pose.yaw_angle = 0
            enemy_pose.distance = 0
            enemy_pose.detected_enemy = 0
            enemy_pose_pub.publish(enemy_pose)
            rospy.loginfo("no enemy in sight!!!!!")
            print ('self pose: %s\t%s\t' % (robot_pose['x'], robot_pose['y']))
            print ('enemy pose: %s\t%s\t\n' % (enemy_point_.point.x, enemy_point_.point.y))
            
        rate.sleep()