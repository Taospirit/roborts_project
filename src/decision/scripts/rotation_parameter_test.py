#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import rospkg
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from battle_env import BattleEnv
from controller import Controller
from decision.msg import EnemyPos
from roborts_msgs.msg import GimbalAngle, GimbalRate, ShootInfo, ShootState, TwistAccel

class TwistControl():
    def __init__(self, move_x, move_y, move_z, angular_z):
        self.Twist = Twist()
        self.Twist.linear.x = move_x
        self.Twist.linear.y = move_y
        self.Twist.linear.z = move_z
        self.Twist.angular.x = 0
        self.Twist.angular.y = 0
        self.Twist.angular.z = angular_z

def normalizeTheta(theta): # 将角度degree归一化为[-pi, pi]
    return math.degrees(math.atan2(math.sin(math.radians(theta)), math.cos(math.radians(theta))))

def TwitsRotation(goal_theta, error_theta = 5, K =3, theta1 = 90, K1 = 4, theta2 = 60, K2 = 6, theta3 = 20, K3 = 9): # 单位是度
    goal_theta = normalizeTheta(goal_theta)

    while True:
        delta_theta = normalizeTheta(goal_theta - env.robot_pose['theta']) # 角度误差
        print ('goal_theta {:.2f}, delta_theta {:.2f}, self_theta {:.2f}'.format(goal_theta, delta_theta, env.robot_pose['theta']))
        if abs(delta_theta) < error_theta:
            break
        if abs(delta_theta) < theta1:
            K = K1
        if abs(delta_theta) < theta2:
            K = K2
        if abs(delta_theta) < theta3:
            K = K3
        v_z = K * math.radians(delta_theta)
        ctrl.send_vel(TwistControl(0, 0, 0, v_z).Twist)

    ctrl.send_vel(TwistControl(0, 0, 0, 0).Twist)


if __name__ == '__main__':
    rospy.init_node('rotation_test')
    rate = rospy.Rate(50)
    ctrl = Controller()
    env = BattleEnv()
    # tflistener = tf.TransformListener()
    ctrl.global_path_planner_action_client.wait_for_server(rospy.Duration(0.5))
    ctrl.local_path_planner_action_client.wait_for_server(rospy.Duration(0.5))
    rospy.sleep(0.5)

    TwitsRotation(input())
    print ('stop')

        

