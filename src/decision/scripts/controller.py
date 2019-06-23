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
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        vel_acc_pub = rospy.Publisher(
            'cmd_vel_acc', TwistAccel, queue_size=1)

        # gimbal related publisher
        gimbal_angle_pub = rospy.Publisher(
            'cmd_gimbal_angle', GimbalAngle, queue_size=1)
        gimbal_angle = GimbalAngle()
        gimbal_rate_pub = rospy.Publisher(
            'cmd_gimbal_rate', GimbalRate, queue_size=1)

        # shoot related publisher
        shoot_state_pub = rospy.Publisher(
            'shoot_state', ShootState, queue_size=1)
        shoot_info_pub = rospy.Publisher(
            'shoot_info', ShootInfo, queue_size=1)

        # shoot command
        cmd_shoot_client = rospy.ServiceProxy('cmd_shoot', ShootCmd)
        try:
            rospy.wait_for_service('cmd_shoot', 0.5)
        except:
            rospy.logwarn('no cmd_shoot service')

        # gimbal mode swtich
        set_gimbal_mode_client = rospy.ServiceProxy(
            'set_gimbal_mode', GimbalMode)
        gimbal_mode = GimbalMode()
        try:
            rospy.wait_for_service('set_gimbal_mode', 0.5)
        except:
            rospy.logwarn('no set_gimbal_mode service')

        # chassis mode switch
        set_chassis_mode_client = rospy.ServiceProxy(
        'set_chassis_mode', ChassisMode)
        chassis_mode = ChassisMode()
        try:
            rospy.wait_for_service('set_chassis_mode', 0.5)
        except:
            rospy.logwarn('no set_chassis_mode service')

        # fric wheel command
        cmd_fric_wheel_client = rospy.ServiceProxy(
            'cmd_fric_wheel', FricWhl)
        fric_wheel_cmd = FricWhl()
        try:
            rospy.wait_for_service('cmd_fric_wheel', 0.5)
        except:
            rospy.logwarn('no cmd_fric_wheel service')
            
        # cmd_fric_wheel_client(True)

        # path planner action to execute the navigation goal
        global_path_planner_action_client = actionlib.SimpleActionClient(
            'global_planner_node_action', GlobalPlannerAction)
        local_path_planner_action_client = actionlib.SimpleActionClient(
            'local_planner_node_action', LocalPlannerAction)
        local_planner_goal = LocalPlannerGoal

    # send vel
    def send_vel(self, vel):
        #rospy.loginfo('send velocity')
        vel_pub.publish(vel)

    # send vel acc
    def send_vel_acc(self, vel_acc):
        rospy.loginfo('send velocity acceleration')
        vel_acc_pub.publish(vel_acc)

    # send gimbal angle
    def send_gimbal_angle(self, gimbal_angle, yaw_mode=False):
        rospy.loginfo('send gimbal angle')
        gimbal_angle.yaw_angle = gimbal_angle
        gimbal_angle.yaw_mode = yaw_mode
        gimbal_angle_pub.publish(gimbal_angle)

    # send gimbal rate
    def send_gimbal_rate(self, gimbal_rate):
        rospy.loginfo('send gimbal rate')
        gimbal_rate_pub.publish(gimbal_rate)

    # send shoot info
    def send_shoot_info(self, shoot_info):
        #rospy.loginfo('send shoot information')
        shoot_info_pub.publish(shoot_info)

    # send shoot state
    def send_shoot_state(self, shoot_state):
        rospy.loginfo('send shoot state')
        shoot_state_pub.publish(shoot_state)

    # shoot command
    def shoot(self, cmd_shoot):
        #rospy.loginfo('send shoot command')
        cmd_shoot_client(cmd_shoot.mode, cmd_shoot.number)

    # gimbal mode swtich
    def gimbal_mode_switch(self, gimbal_mode):
        # rospy.loginfo('switch gimbal mode')
        # gimbal_mode = gimbal_mode
        while not set_gimbal_mode_client(gimbal_mode):
            print ('----set_gimbal_mode false,retry-----')
            pass
        
    # chassis mode switch
    def chassis_mode_switch(self, chassis_mode):
        #rospy.loginfo('switch chassis mode')
        # chassis_mode = chassis_mode
        while not set_chassis_mode_client(chassis_mode):
            print ('----set_chassis_mode false,retry-----')
            pass

    # fric wheel
    def fric_wheel(self, fric_wheel):
        rospy.loginfo('swtich fric wheel mode')
        fric_wheel_cmd = fric_wheel
        cmd_fric_wheel_client(fric_wheel_cmd)

    # send navigation goal
    def send_goal(self, goal):
        #rospy.loginfo('send navigation goal')
        global_path_planner_action_client.send_goal(
            goal, feedback_cb=global_path_planner_feedback_cb)
        global_path_planner_action_client.wait_for_result()

    # global path planner feedback
    def global_path_planner_feedback_cb(self, feedback):
        if len(feedback.path.poses) != 0:
            local_planner_goal.route = feedback.path
            local_path_planner_action_client.send_goal_and_wait(
                local_planner_goal)

if __name__ == "__main__":
    rospy.init_node('Controller')
    ctrl = Controller()
    env = BattleEnv()

    navgoal = GlobalPlannerGoal()
    navgoal.goal.header.frame_id = 'map'
    print ('wait topic message: /move_base_simple/goal ...')

    def getRvizPoseCallback(data):
        navgoal.goal.pose.position.x = data.pose.position.x
        navgoal.goal.pose.position.y = data.pose.position.y
        navgoal.goal.pose.position.z = data.pose.position.z
        navgoal.goal.pose.orientation.x = data.pose.orientation.x
        navgoal.goal.pose.orientation.y = data.pose.orientation.y
        navgoal.goal.pose.orientation.z = data.pose.orientation.z
        navgoal.goal.pose.orientation.w = data.pose.orientation.w
        print ('send_goal!')
        env.send_goal(navgoal)

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, getRvizPoseCallback, queue_size=1)

    

    # while not rospy.is_shutdown():





    rospy.spin()

