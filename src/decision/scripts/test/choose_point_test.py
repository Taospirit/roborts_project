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
def choosePoint(self_pose, enemy_pose, dist = 1.5): # choose point from robot0 pose and robot1 pose
    dist_min = float('inf')
    goal_x, goal_y = 0, 0
    for t in range(-180, 180, 2):
        x, y = enemy_pose[0] + dist * math.cos(math.radians(t)), enemy_pose[1] + dist * math.sin(math.radians(t))
       
        if not isAviable(x, y):
            continue
        if not isReachable(x, y, enemy_pose[0], enemy_pose[1]):
            continue
        if isNear(x, y):
            continue
        # print (x, y)
        dist_tmp = math.hypot(self_pose[0] - x, self_pose[1] - y)
        if dist_tmp < dist_min:
            dist_min = dist_tmp
            goal_x, goal_y = x, y
    return goal_x, goal_y

def isAviable(x, y):
    x_i = int(np.around(x * 200))
    y_i = int(np.around(y * 200))
    if x_i >= 1600 or x_i <= 0:
        # print ('x out')
        return False
    if y_i >= 1000 or y_i <= 0:
        # print ('y out')
        return False
    if map[1000 - y_i, x_i] < 255:
        # print ('x y in ')
        return False
    else:
        return True

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
    # if map[y_i - offset, x_i - offset] < 255 or map[y_i - offset, x_i + offset] < 255:
    #     return True
    # if map[y_i + offset, x_i - offset] < 255 or map[y_i + offset, x_i + offset] < 255:
    #     return True

    # return False

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

# Follow
def follow_goal(enemy_x, enemy_y, robot_x, robot_y):
    if isReachable(enemy_x, enemy_y, robot_x, robot_y) == False:
        return 0, 0
    change_angle_list = [0, 0, 5, -5, 10, -10, -17, 17, 24, -24, 32, -32, 40, -40, 50, -50, 60, -60]
    enemy_dist = math.hypot(enemy_y - robot_y, enemy_x - robot_x)
    enemy_yaw = math.degrees(math.atan2(enemy_y - robot_y, enemy_x - robot_x))
    print(enemy_yaw)
    count = 0
    change_angle = 0
    goal_x = 0
    goal_y = 0
    goal_yaw = 0

    if close_obstacle(enemy_x, enemy_x, robot_x, robot_y, 0.3) == True:
        keep_length = 0.7
    elif enemy_dist > 0 and enemy_dist < 2.6:
        keep_length = 1.2 # 跟随距离
    else:
        keep_length = 0

    while isAviable(goal_x, goal_y) == False:# or isNear(goal_x, goal_y) == False:
        count += 1
        if count > (len(change_angle_list) - 1):
            return 0, 0   # 无法跟随         
        change_angle = change_angle_list[count] # 旋转角度 radian
                
        yaw = enemy_yaw + change_angle - 180
        goal_x = keep_length * math.cos(math.radians(yaw)) + enemy_x
        goal_y = keep_length * math.sin(math.radians(yaw)) + enemy_y
        goal_yaw = yaw - 180

        if isReachable(enemy_x, enemy_y, goal_x, goal_y) == False or obstacle_dis(goal_x, goal_y,
                0.4) == False:
            goal_x = 0
            goal_y = 0
    print(goal_yaw)
    return goal_x, goal_y

def rescue_follow_goal(enemy_x, enemy_y, robot_x, robot_y):
    enemy_yaw = math.atan2(enemy_y - robot_y, enemy_x - robot_x)
    if enemy_yaw >= 90:
        goal_x = robot_x
        goal_y = enemy_y
        goal_yaw = 178
        if isReachable(goal_x, goal_y, robot_x, robot_y) == False:
            goal_x = enemy_x
            goal_y = robot_y
            goal_yaw = 90
    elif enemy_yaw >= 0:
        goal_x = robot_x
        goal_y = enemy_y
        goal_yaw = 0.01
        if isReachable(goal_x, goal_y, robot_x, robot_y) == False:
            goal_x = enemy_x
            goal_y = robot_y
            goal_yaw = 90
    elif enemy_yaw >= -90:
        goal_x = robot_x
        goal_y = enemy_y
        goal_yaw = 0.01
        if isReachable(goal_x, goal_y, robot_x, robot_y) == False:
            goal_x = enemy_x
            goal_y = robot_y
            goal_yaw = -90         
    else:
        goal_x = robot_x
        goal_y = enemy_y
        goal_yaw = 178
        if isReachable(goal_x, goal_y, robot_x, robot_y) == False:
            goal_x = enemy_x
            goal_y = robot_y
            goal_yaw = -90
    if isAviable(goal_x, goal_y) == False:# or isNear(goal_x, goal_y) == False:
        return goal_x, goal_y
    else:
        return goal_x, goal_y

def obstacle_dis(robot_x, robot_y, dis_set):
    dis_min = 10
    for i in obstacle_conner:
        dis = math.hypot(robot_x - i[0], robot_y - i[1])
        if dis < dis_min:
            dis_min = dis
    if dis_min < dis_set:
        return False
    else:
        return True

def close_obstacle(enemy_x, enemy_y, robot_x, robot_y, dis_set):
    dis_min = 10
    for i in obstacle_conner:
        if judge(i[0], i[1], enemy_x, enemy_y, robot_x, robot_y):
            dis = dis_point_to_line(i[0], i[1], enemy_x, enemy_y, robot_x, robot_y)
            if dis_min > dis:
                dis_min = dis
    if dis_min < dis_set:
        return True
    else:
        return False

def dis_point_to_line(pointX, pointY, lineX1, lineY1, lineX2, lineY2): # 一般式
    a = lineY2 - lineY1
    b = lineX1 - lineX2
    c = lineX2 * lineY1 - lineX1 * lineY2
    dis = (math.fabs(a*pointX+b*pointY+c)) / (math.pow(a*a+b*b,0.5))
    return dis

def judge(pointX, pointY, pointX1, pointY1, pointX2, pointY2):
    if (pointX >= pointX1 and pointX <= pointX2) or (pointX <= pointX1 and pointX >= pointX2):
        if (pointY >= pointY1 and pointY <= pointY2) or (pointY <= pointY1 and pointY >= pointY2):
            return True
        else:
            return False
    else:
        return False

if __name__ == "__main__":
    rospy.init_node('dual_choose_test')
    rate = rospy.Rate(20)

    robot0_pose = {'x':1, 'y':1, 'theta':0}
    robot1_pose = {'x':1, 'y':1, 'theta':0}

    tf0 = tf.TransformListener()
    tf1 = tf.TransformListener()

    point_pub = rospy.Publisher('point_choosed', PointStamped, queue_size=1)
    point_pub_1 = rospy.Publisher('point_1', PointStamped, queue_size=1)


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

        point_pub_.point.x, point_pub_.point.y = follow_goal(robot1_pose['x'], robot1_pose['y'], robot0_pose['x'], robot0_pose['y'])
        # if point_pub_.point.x == 0:
        #     point_pub_.point.x, point_pub_.point.y = rescue_follow_goal(robot1_pose['x'], robot1_pose['y'], robot0_pose['x'], robot0_pose['y'])

        # point_pub_.point.x, point_pub_.point.y = choosePoint([robot0_pose['x'], robot0_pose['y']], [robot1_pose['x'], robot1_pose['y']])
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
        