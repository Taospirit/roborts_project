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
'''
x: 30.36
y: 24.35
'''

class humanPos:
    def __init__(self):
        self.bridge = CvBridge()
        self.center_x, self.center_y, self.offset_x, self.offset_y = 0, 0, 0, 0
        self.visual_angle_x, self.visual_angle_y, self.angle_pitch= radians(30.36), radians(24.35), radians(46.7)
        self.image_shape = [640, 480]
        self.not_detect_count, self.not_detect_set = 0, 15

        self.human_position = HumanPosition()
        self.human_height = 0
        self.human_min_dist = 1000 # mm
        self.human_max_dist = 3000

        self.last_distance = 0
        self.last_angle = 0

        rospy.Subscriber("face_position", FacePosition, self.getFacePosition, queue_size=1)
        rospy.Subscriber("supply_distance", SupplyDistance, self.faceDepthCallback, queue_size=1)
        # rospy.Subscriber("/depth/image_rect_raw", Image, self.getFaceDepth, queue_size=1)
        self.human_position_pub = rospy.Publisher("human_position", HumanPosition, queue_size=1)
        
    def getFacePosition(self, data):
        self.center_x = data.face_center_x
        self.center_y = data.face_center_y
        self.offset_x = data.face_offset_x
        self.offset_y = data.face_offset_y
        # print ("center is {:.2f} {:.2f}".format(self.center_x, self.center_y))

    def faceDepthCallback(self, data):
        if data.supply_distance == 0:
            self.human_position.human_dist = self.last_distance
        elif data.supply_distance < self.human_min_dist or data.supply_distance > self.human_max_dist:
            self.human_position.human_dist = self.last_distance
        else:
            self.human_position.human_dist = data.supply_distance # min meter
        
        # self.human_position.human_dist = (data.supply_distance if (data.supply_distance != 0 ) else self.last_distance)
        
        if self.center_x == 0:
            self.human_position.human_angle = self.last_angle
            self.not_detect_count += 1
        else:
            delta_pixel_x = self.image_shape[0] / 2 - self.center_x
            delta_pixel_y = self.image_shape[1] / 2 - self.center_y
            angle_x = atan2(delta_pixel_x * tan(self.visual_angle_x), self.image_shape[0] / 2)
            angle_y = atan2(delta_pixel_y * tan(self.visual_angle_y), self.image_shape[1] / 2)

            print ("-----------------------------")
            print ("angle_x is {:.3f}, angle_y is {:.3f}".format(angle_x, angle_y))

            self.human_position.human_angle = degrees(atan2(tan(angle_x) * cos(angle_y), cos(angle_y + self.angle_pitch)))
            self.human_height = self.human_position.human_dist / cos(angle_y) * sin(angle_y + self.angle_pitch)
            self.not_detect_count = 0

        if self.not_detect_count > self.not_detect_set:
            self.human_position.human_angle, self.human_position.human_dist = 0, 0

        print ("Height is {}".format(self.human_height))
        print ('')
        # rospy.loginfo("Height is {}".format(self.human_position.human_dist / cos(angle_y) * sin(angle_y + self.angle_pitch)))
        self.human_position_pub.publish(self.human_position)

        self.last_distance = self.human_position.human_dist
        self.last_angle = self.human_position.human_angle
        

if __name__ == "__main__":
    rospy.init_node('human_position')
    try:
        rospy.loginfo('start get human_info from face_info...')
        humanPos()
        rospy.spin()
    except e:
        print (e)