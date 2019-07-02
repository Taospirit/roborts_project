#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import cv2
import numpy as np
import copy
from math import sin, cos, tan, atan2, radians, degrees
from sensor_msgs.msg import Image, RegionOfInterest, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from roborts_msgs.msg import SupplyDistance, FacePosition, HumanPosition
from geometry_msgs.msg import Twist, Vector3
from controller import Controller
from roborts_msgs.msg import GlobalPlannerGoal
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class TwistControl():
    def __init__(self, speed_x, speed_y, rotation_z):
        self.Twist = Twist()
        self.Twist.linear = Vector3(speed_x, speed_y, 0)
        self.Twist.angular = Vector3(0, 0, rotation_z)

class moveControl():
    def __init__(self):
        self.ctrl = Controller()
        self.dist, self.angle = 0, 0

        self.max_angle_error = 30
        self.min_rotation_speed, self.max_rotation_speed = 0.1, 0.6
        self.angle_error = 10
        self.need_rotation = True

        self.max_dist_error = 500
        self.min_move_speed, self.max_move_speed = 0.1, 0.7
        self.dist_error = 150
        self.goal_dist = 1500

        self.integrator_z = 0
        self.last_time = 0
        self.last_error = 0

        self.Kp_z = 1
        self.Ki_z = 0
        self.Kd_z = 0

        self.dist_list = []
        self.angle_list = []
        self.info_size = 20

        self.laser_dist_min = float('inf')
        self.laser_dist_set = 0
        rospy.Subscriber("human_position", HumanPosition, self.positionCallback, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.laserCallback, queue_size=1)


    def laserCallback(self, scan_data):
        ranges = np.arrray(scan_data.ranges)
        self.laser_dist_min = ranges.min()
        print ("laser_dist_min is {}".format(self.laser_dist_min))

    def positionCallback(self, data):
        self.dist_list.append(data.human_dist)
        self.angle_list.append(data.human_angle)

        while len(self.dist_list) > self.info_size:
            self.dist_list.pop(0)
        while len(self.angle_list) > self.info_size:
            self.angle_list.pop(0)

        self.dist, self.angle = np.mean(self.dist_list), np.mean(self.angle_list)
        print ("control dist {:.3f} and angle {:.3f} ".format(self.dist, self.angle))

        if self.laser_dist_min > self.laser_dist_set:
            print ('-----use twist control-----')
            self.twistMove()
        else:
            print ('-----use planner model-----')
            self.navigation()

    def twistMove(self):
        if (abs(self.angle) > self.angle_error):
            self.need_rotation = True
            sign = (1 if (self.angle > 0) else -1)
            v_z = self.linearP_z()
            self.ctrl.send_vel(TwistControl(0, 0, sign * v_z).Twist)
        else:
            self.need_rotation = False
            self.ctrl.send_vel(TwistControl(0, 0, 0).Twist)
            print ("stop rotation for {:.3f}".format(self.angle))

        if not self.need_rotation and self.dist != 0:
            dist_error = self.goal_dist - self.dist
            print ("delta_x is {:.3f}".format(dist_error))
            if (abs(dist_error) > self.dist_error):
                sign = (1 if (dist_error > 0) else -1)
                v_x = self.linearP_x()
                self.ctrl.send_vel(TwistControl(sign * v_x, 0, 0).Twist)

    def linearP_z(self):
        k_ = (self.max_rotation_speed - self.min_rotation_speed) / (self.max_angle_error - self.angle_error) # (1 - 0.5)/(30 - 4)
        return (abs(self.angle) - self.angle_error) * k_ + self.min_rotation_speed # (x - 5) * k + 0.1
        
    def linearP_x(self):
        k_ = (self.max_move_speed - self.min_move_speed) / (self.max_dist_error - self.dist_error)
        return (abs(self.goal_dist - self.dist) - self.dist_error) * k_ + self.min_move_speed

    def simplePID(self, sign):
        error = self.angle
        P = error
        current_time = time.clock()
        delta_time = current_time - self.last_time

        self.integrator_z += error * delta_time
        I = self.integrator_z
        D = (error - self.last_error) / delta_time
        self.last_error = error
        self.last_time = current_time

        return self.Kp_z*P + self.Ki_z*I + self.Kd_z*D

    def navigation(self):
        goal = GlobalPlannerGoal()
        goal_x, goal_y, goal_theta = 0, 0, 0
        quat = tf.transformations.quaternion_from_euler(0, 0, radians(goal_theta))
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = goal_x
        goal.goal.pose.position.y = goal_y
        goal.goal.pose.position.z = 0
        goal.goal.pose.orientation.x = quat[0]
        goal.goal.pose.orientation.y = quat[1]
        goal.goal.pose.orientation.z = quat[2]
        goal.goal.pose.orientation.w = quat[3]
        print ("send goal point {:.3f} {:.3f} {:.3f}".format(goal_x, goal_y, goal_theta)
        # self.ctrl.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node("robot_move")
    # try:
    rospy.loginfo('get info for detect_face and position...')
    moveControl()
    rospy.spin()
    # except e:
        # print (e)