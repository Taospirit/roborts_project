#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import copy
from math import sin, cos, tan, atan2, radians, degrees
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from roborts_msgs.msg import SupplyDistance, FacePosition, HumanPosition
from geometry_msgs.msg import Twist, Vector3
from controller import Controller


class TwistControl():
    def __init__(self, speed_x, speed_y, rotation_z):
        self.Twist = Twist()
        self.Twist.linear = Vector3(speed_x, speed_y, 0)
        self.Twist.angular = Vector3(0, 0, rotation_z)

        # self.Twist.linear.x = speed_x
        # self.Twist.linear.y = speed_y
        # self.Twist.linear.z = 0
        # self.Twist.angular.x = 0
        # self.Twist.angular.y = 0
        # self.Twist.angular.z = rotation_z

class twistMove():
    def __init__(self):
        self.ctrl = Controller()
        self.dist, self.angle = 0, 0

        self.max_angle = 30
        self.min_rotation_speed, self.max_rotation_speed = 0.5, 1
        self.angle_error = 4

        self.min_move_speed, self.max_move_speed = 0.2, 0.6
        self.dist_error = 200

        self.r_integrator = 0

        rospy.Subscriber("human_position", HumanPosition, self.positionCallback, queue_size=1)

    def positionCallback(self, data):
        self.dist, self.angle = data.human_dist, data.human_angle

        if (abs(self.angle) > self.angle_error):
            sign = (1 if (self.angle > 0) else -1)
            k_ = (self.max_rotation_speed - self.min_rotation_speed) / (self.max_angle - self.angle_error) # (1 - 0.5)/(30 - 4)
            v_z = (abs(self.angle) - self.angle_error) * k_ + self.min_rotation_speed # (x - 5) * k + 0.1
            
            self.ctrl.send_vel(TwistControl(0, 0, sign * v_z).Twist)
            print ("delta_angel is {:.3f}".format(self.angle))
        else:
            self.ctrl.send_vel(TwistControl(0, 0, 0).Twist)
            print ("------stop rotation for {}------".format(self.angle))


if __name__ == "__main__":
    rospy.init_node("robot_move")
    # try:
    rospy.loginfo('get info for detect_face and position...')
    twistMove()
    rospy.spin()
    # except e:
        # print (e)