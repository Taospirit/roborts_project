#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import math
import tf
import numpy as np
import os
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf import TransformListener
from PIL import Image
from tf.transformations import euler_from_quaternion


# robot_0 is me, robot_1 is enemy
def choosePoint(dist = 1.0): # choose point from robot0 pose and robot1 pose
    choose_point = []

    for t in range(-180, 180, 1):
        x, y = robot1_pose['x'] + dist * math.cos(math.radians(t)), robot1_pose['y'] + dist * math.sin(math.radians(t))
        if isNear(x, y):
            continue
        if not isReachable(x, y, robot1_pose['x'], robot1_pose['y']):
            continue
        
        
        choose_point.append([x, y])

    return getCloestPoint(robot0_pose['x'], robot0_pose['y'], choose_point)

def getCloestPoint(x, y, open_list):
    if len(open_list) == 0:
        raise Exception('the list is NONE')
    dist_min = float('inf')
    p_min = []
    for p in open_list:
        dist = math.hypot(x - p[0], y - p[1])
        if dist < dist_min:
            dist_min = dist
            p_min = p
    return p_min[0], p_min[1]

def isNear(x, y, safe_dist = 0.3):
    offset = int(safe_dist * 200)
    x_i = int(np.around(x * 200))
    y_i = 1000 -int(np.around(y * 200))
    if y_i + offset >= 1000 or y_i - offset <= 0:
        return True
    if x_i + offset >= 1600 or x_i + offset <= 0:
        return True
    
    for f in range(-offset, offset, 1):
        if map[y_i - offset, x_i + f] < 255:
            return True
        if map[y_i + offset, x_i + f] < 255:
            return True
        if map[y_i + f, x_i - offset] < 255:
            return True
        if map[y_i + f, x_i + offset] < 255:
            return True
    return False

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
                # print ('false_unreachable_in_line')
                return False
    elif self_y_i > enemy_y_i:
        self_x_i, enemy_x_i = enemy_x_i, self_x_i
        for y in range(self_y_i, enemy_y_i):
            if map[1000 - y, self_x_i] < 255:
                # print ('fasle_unreachable_in_y')
                return False
    # print ('isreachable')
    return True

if __name__ == "__main__":
    rospy.init_node('dual_choose_test')
    rate = rospy.Rate(20)

    robot0_pose = {'x':1, 'y':1, 'theta':0}
    robot1_pose = {'x':1, 'y':1, 'theta':0}

    tf0 = tf.TransformListener()
    tf1 = tf.TransformListener()

    point_pub = rospy.Publisher('search_choosed', PointStamped, queue_size=1)
    choose_point = []


    point_pub_ = PointStamped()
    point_pub_.header.frame_id = 'map'
    point_pub_.header.stamp = rospy.Time()
    point_pub_.point.z = 0

    map = np.array(Image.open("{}/icra2019_s.pgm".format(os.path.split(os.path.realpath(__file__))[0]))) 

    obstacle_conner = [[1.4, 1.4], [1.4, 2.4], [1.65, 2.4], [1.65, 1.4],[1.2, 3.75], [1.2, 4], [2.2, 4], [2.2, 3.75],
                    [3.5, 2.375], [3.5, 2.625], [4.5, 2.625], [4.5, 2.375], [6.35, 2.6], [6.35, 3.6], [6.6, 3.6], [6.6, 2.6],
                    [5.8, 1], [5.8, 1.25], [6.8, 1.25], [6.8, 1], [4.5, 4], [4.75, 4], [3.25, 1], [3.5, 1]]
    while not rospy.is_shutdown():
        try:
            t0, q0 = tf0.lookupTransform('map', 'robot_0/base_link', rospy.Time(0))
            t1, q1 = tf1.lookupTransform('map', 'robot_1/base_link', rospy.Time(0))
            robot0_pose['x'], robot0_pose['y'] = t0[0], t0[1]
            robot1_pose['x'], robot1_pose['y'] = t1[0], t1[1]
            
            angle0 = euler_from_quaternion((q0[0], q0[1], q0[2], q0[3]))
            angle1 = euler_from_quaternion((q1[0], q1[1], q1[2], q1[3]))
            robot0_pose['theta'], robot1_pose['theta'] = math.degrees(angle0[2]), math.degrees(angle1[2])

        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as err:
            rospy.logwarn('no robot tf')
            rospy.logwarn(err)

        point_pub_.point.x, point_pub_.point.y = choosePoint(2)
        point_pub.publish(point_pub_)

        # dist_0 = float('inf')
        # for p in obstacle_conner:
        #     dist_tmp = math.hypot(point_pub_.point.x - p[0], point_pub_.point.y - p[1])
        #     if dist_tmp < dist_0:
        #         dist_0 = dist_tmp
        # print (dist_0)

        # dist_1 = float('inf')
        # for p in obstacle_conner:
        #     dist_tmp = math.hypot(robot1_pose['x'] - p[0], robot1_pose['y'] - p[1])
        #     if dist_tmp < dist_1:
        #         dist_1 = dist_tmp
        # print (dist_1)

        print('\nrobot0_pose {:.2f},{:.2f} with {:.2f}, robot1_pose {:.2f},{:.2f} with {:.2f}'.format(robot0_pose['x'], robot0_pose['y'], robot0_pose['theta'], robot1_pose['x'], robot1_pose['y'], robot1_pose['theta']))
        print('point is {}, {}'.format(point_pub_.point.x, point_pub_.point.y))
        rate.sleep()    
        