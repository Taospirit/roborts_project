#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import random
import math
import rospy
import tf
import numpy as np
from PIL import Image
from roborts_msgs.msg import ArmorDetectionAction, ArmorDetectionGoal, ArmorDetection
#from decision.msg import EnemyPos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

map = np.array(
    Image.open("{}/icra2019_s.pgm".format(
        os.path.split(os.path.realpath(__file__))[0])))


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

def normalizeTheta(theta):  # 将角度归一化为[-180, 180]
    return math.degrees(
        math.atan2(
            math.sin(math.radians(theta)), math.cos(math.radians(theta))))

def isInSight(enemy_x, enemy_y, self_x, self_y):
    # 转化到图像坐标系
    enemy_x_i = int(np.around(enemy_x * 200))
    enemy_y_i = int(np.around(enemy_y * 200))
    self_x_i = int(np.around(self_x * 200))
    self_y_i = int(np.around(self_y * 200))
    print(self_x_i, self_y_i, enemy_x_i, enemy_y_i)
    if enemy_x_i >= 1600 or enemy_x_i <= 0 or enemy_y_i >= 1000 or enemy_y_i <= 0:
        return False, 0
    if self_x_i >= 1600 or self_x_i <= 0 or self_y_i >= 1000 or self_y_i <= 0:
        return False, 0

    A = np.mat([[self_x_i, 1], [enemy_x_i, 1]])
    B = np.mat([[self_y_i], [enemy_y_i]])

    if np.linalg.det(A) != 0:
        _k_b_ = np.linalg.solve(A, B)
        _k = _k_b_[0, 0]
        _b = _k_b_[1, 0]
        if self_x_i > enemy_x_i:
            self_x_i, enemy_x_i = enemy_x_i, self_x_i
        for x in range(self_x_i, enemy_x_i):
            y = int(np.around(_k * x + _b))
            if map[1000 - y, x] < 255:
                print('false_unreachable_in_line')
                return False, 0
    elif self_y_i > enemy_y_i:
        self_x_i, enemy_x_i = enemy_x_i, self_x_i
        for y in range(self_y_i, enemy_y_i):
            if map[1000 - y, self_x_i] < 255:
                print('fasle_unreachable_in_y')
                return False, 0
    print('is_insight')
    dist = math.hypot(enemy_x - self_x, enemy_y - self_y)
    return True, dist

def isInFrontSight(enemy_x, enemy_y, self_x, self_y, self_theta, visible_dist, visible_theta):  # yaw单位是角度
    dist = math.hypot(enemy_x - self_x, enemy_y - self_y)
    # 单位都是角度
    theta_enemy = math.degrees(math.atan2(enemy_y - self_y, enemy_x - self_x))
    theta_enemy_diff = normalizeTheta(theta_enemy - normalizeTheta(self_theta)) 
    
    if abs(theta_enemy_diff) > visible_theta:
        # print('false_theta_out %s, theat_diff is %s' % (abs(theta_enemy_diff), theta_enemy_diff)
        print('dist is %s, false_theta_diff is %s' % (dist, theta_enemy_diff))
        return False, theta_enemy_diff
    if dist > visible_dist:
        # print('false_dist_out %s' % dist)
        print('false_dist is %s, theta_diff is %s' % (dist, theta_enemy_diff))
        return False, theta_enemy_diff

    print('dist is %s, theta_diff is %s' % (dist, theta_enemy_diff))
    return True, theta_enemy_diff

if __name__ == '__main__':
    rospy.init_node('enemy_detect')
    rate = rospy.Rate(100)

    robot_pose = {'x': 0, 'y': 0, 'theta': 0}
    enemy_pos = {'x': 0, 'y': 0}
    tflistener = tf.TransformListener()

    node_ns = rospy.get_namespace()
    if node_ns == "/robot_0/":
        enemy_name = "/robot_1/"
    elif node_ns == "/robot_1/":
        enemy_name = "/robot_0/"
    else:
        enemy_name = ""

    rospy.Subscriber('robot_pose', Odometry, getSelfPoseCallback, queue_size=1)

    enemy_pose = ArmorDetection()
    enemy_pose_pub = rospy.Publisher(
        'armor_detection_info', ArmorDetection, queue_size=1)

    # STEP = 0.1  # 敌人移动速度
    V_DIST = 4  # 视觉检测有效距离
    V_THETA_FRONT = 50  # 前摄像头视觉检测左右有效角度

    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0)
            # tflistener.waitForTransform("/map", enemy_name + 'base_link', now, rospy.Duration(1))
            t, q = tflistener.lookupTransform("/map", enemy_name + 'base_link', now)
            enemy_pos['x'] = t[0]
            enemy_pos['y'] = t[1]
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as err:
            rospy.logwarn('no simulation enemy tf')
            rospy.logwarn(err)

        _isInFrontSight, _theta = isInFrontSight(enemy_pos['x'], enemy_pos['y'], robot_pose['x'], robot_pose['y'],
            robot_pose['theta'], V_DIST, V_THETA_FRONT)
        
        _isInSight, _dist = isInSight(enemy_pos['x'], enemy_pos['y'], robot_pose['x'], robot_pose['y'])

        if _isInSight:
            enemy_pose.yaw_angle = math.radians(_theta)
            enemy_pose.distance = _dist * 1000
            enemy_pose.detected_enemy = True
            if _isInFrontSight:
                enemy_pose.detected_by_front = True
            else:
                enemy_pose.detected_by_front = False

            enemy_pose_pub.publish(enemy_pose)
            rospy.logwarn("enemy in sight! yaw_angle_theta is {:.2f}, front:{}".format(_theta, enemy_pose.detected_by_front))
            rospy.loginfo('self pose: {:.2f}, {:.2f}'.format(robot_pose['x'], robot_pose['y']))
            rospy.loginfo('enemy pose: {:.2f}, {:.2f}\n'.format(enemy_pos['x'], enemy_pos['y']))
        else:
            enemy_pose.yaw_angle = 0
            enemy_pose.distance = 0
            enemy_pose.detected_enemy = False
            enemy_pose.detected_by_front = False
            enemy_pose_pub.publish(enemy_pose)
            rospy.loginfo("no enemy in sight!!!!!")
            rospy.loginfo('self pose: {:.2f}, {:.2f}'.format(robot_pose['x'], robot_pose['y']))
            rospy.loginfo('enemy pose: {:.2f}, {:.2f}\n'.format(enemy_pos['x'], enemy_pos['y']))

        rate.sleep()
    rospy.spin()
