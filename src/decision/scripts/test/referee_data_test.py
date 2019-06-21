#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import random
import numpy as np
# import google.protobuf as pb
import rospy
import rospkg
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# from pi_trees_ros.pi_trees_ros import *
from battle_env import BattleEnv
from controller import Controller
from decision.msg import EnemyPos
from roborts_msgs.msg import GimbalAngle, GimbalRate, ShootInfo, ShootState, TwistAccel


if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10)
    env = BattleEnv()
    # tflistener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        print ('\n****************************************************')
        print ('BounsStatus: red_bonus is {}, blue_bonus is {}'.format(env.RED_BONUS_STATUS, env.BLUE_BONUS_STATUS))
        print ('GameStatus: game_status is {}, remaining_time is {}'.format(env.GAME_STATUS, env.REMAINING_TIME))
        print ('GameSurvivor: {}')
        
        print ('\nProjectileSupply: supply(bool) is {}'.format(env.PROJECTILE_SUPPLY))
        print ('RobotBonus: bonus(bool) is {}'.format(env.ROBOT_BONUS))
        
        print ('\n----------------RobotHeat:------------------')
        print ('-----------CHASSIS_VOLT: {}------------------'.format(env.CHASSIS_VOLT))
        print ('-----------CHASSIS_CURRENT: {}------------------'.format(env.CHASSIS_CURRENT))
        print ('-----------CHASSIS_POWER: {}------------------'.format(env.CHASSIS_POWER))
        print ('-----------CHASSIS_POWER_BUFFER: {}------------------'.format(env.CHASSIS_POWER_BUFFER))
        print ('-----------SHOOTER_HEAT: {}------------------'.format(env.SHOOTER_HEAT))

        print ('\nRobotShoot: frequency is {}, speed is {}'.format(env.SHOOT_FREQUENCY, env.SHOOT_SPEED))

        print ('\n----------------RobotStatus:------------------')
        print ('-----------REMAIN_HP: {}------------------'.format(env.REMAIN_HP))
        print ('-----------MAX_HP: {}------------------'.format(env.MAX_HP))
        print ('-----------HEAT_COOLING_LIMIT: {}------------------'.format(env.HEAT_COOLING_LIMIT))
        print ('-----------HEAT_COOLING_RATE: {}------------------'.format(env.HEAT_COOLING_RATE))
        print ('-----------GIMBAL_OUTPUT: {}------------------'.format(env.GIMBAL_OUTPUT))
        print ('-----------CHASSIS_OUTPUT: {}------------------'.format(env.CHASSIS_OUTPUT))
        print ('-----------SHOOTER_OUTPUT: {}------------------'.format(env.SHOOTER_OUTPUT))

        print ('\nSupplierStatus: status is {}, 0_close, 1_pre, 2_ing'.format(env.SUPPLIER_STATUS))

        rate.sleep()
