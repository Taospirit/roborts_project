#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import tf
import actionlib
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf import TransformListener
# from roborts_msgs.msg import GlobalPlannerAction, LocalPlannerAction, LocalPlannerGoal, GlobalPlannerFeedback, GlobalPlannerGoal
# from roborts_msgs.msg import ArmorDetectionAction, ArmorDetectionGoal
# from roborts_msgs.msg import GimbalAngle, GimbalRate, ShootInfo, ShootState, TwistAccel
from roborts_msgs.srv import ShootCmd, GimbalMode, FricWhl, ChassisMode
from decision.msg import EnemyPos

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
'''
class which controls the velocity, shoot command, mode switch, navigation goal.
'''
class Controller(object):
    def __init__(self):
        # velocity related publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vel_acc_pub = rospy.Publisher(
            'cmd_vel_acc', TwistAccel, queue_size=1)

        # fric wheel command
        self.cmd_fric_wheel_client = rospy.ServiceProxy(
            'cmd_fric_wheel', FricWhl)
        self.fric_wheel_cmd = FricWhl()
        try:
            rospy.wait_for_service('cmd_fric_wheel', 0.5)
        except:
            rospy.logwarn('no cmd_fric_wheel service')
            
        # cmd_fric_wheel_client(True)

        # path planner action to execute the navigation goal
        self.global_path_planner_action_client = actionlib.SimpleActionClient(
            'global_planner_node_action', GlobalPlannerAction)
        self.local_path_planner_action_client = actionlib.SimpleActionClient(
            'local_planner_node_action', LocalPlannerAction)
        self.local_planner_goal = LocalPlannerGoal

    # send vel
    def send_vel(self, vel):
        #rospy.loginfo('send velocity')
        self.vel_pub.publish(vel)

    # send vel acc
    def send_vel_acc(self, vel_acc):
        rospy.loginfo('send velocity acceleration')
        self.vel_acc_pub.publish(vel_acc)

    # fric wheel
    def fric_wheel(self, fric_wheel):
        rospy.loginfo('swtich fric wheel mode')
        self.fric_wheel_cmd = fric_wheel
        self.cmd_fric_wheel_client(fric_wheel_cmd)

    # send navigation goal
    def send_goal(self, goal):
        #rospy.loginfo('send navigation goal')
        self.global_path_planner_action_client.send_goal(
            goal, feedback_cb=self.global_path_planner_feedback_cb)
        self.global_path_planner_action_client.wait_for_result()

    # global path planner feedback
    def global_path_planner_feedback_cb(self, feedback):
        if len(feedback.path.poses) != 0:
            self.local_planner_goal.route = feedback.path
            self.local_path_planner_action_client.send_goal_and_wait(
                self.local_planner_goal)

if __name__ == "__main__":
    rospy.init_node('Controller')
    ctrl = Controller()
    env = BattleEnv()

    rospy.spin()

