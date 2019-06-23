#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from roborts_msgs.msg import GlobalPlannerAction
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
from geometry_msgs.msg import Twist, PoseStamped
from battle_env import BattleEnv
from controller import Controller

def getRvizPoseCallback(data):
    navgoal.goal.header.frame_id = data.header.frame_id
    navgoal.goal.pose.position.x = data.pose.position.x
    navgoal.goal.pose.position.y = data.pose.position.y
    navgoal.goal.pose.position.z = data.pose.position.z
    navgoal.goal.pose.orientation.x = data.pose.orientation.x
    navgoal.goal.pose.orientation.y = data.pose.orientation.y
    navgoal.goal.pose.orientation.z = data.pose.orientation.z
    navgoal.goal.pose.orientation.w = data.pose.orientation.w
    print ('send_goal!')
    env.send_goal_force(navgoal)

if __name__ == "__main__":
    rospy.init_node('navigate')
    ctrl = Controller()
    env = BattleEnv()

    navgoal = GlobalPlannerGoal()
    print ('wait topic message: /move_base_simple/goal ...')
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, getRvizPoseCallback, queue_size=1)

    rospy.spin()