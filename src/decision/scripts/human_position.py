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
        self.visual_angle_x, self.visual_angle_y, self.angle_pitch= radians(30.36), radians(24.35), radians(40)
        self.image_shape = [640, 480]
        self.face_depth_mean = 0
        self.human_position = HumanPosition()

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
        self.human_position.human_dist = data.supply_distance # meter
        
        delta_pixel_x = self.center_x - self.image_shape[0] / 2
        delta_pixel_y = self.center_y - self.image_shape[1] / 2
        angle_x = atan2(delta_pixel_x * tan(self.visual_angle_x), self.image_shape[0])
        angle_y = atan2(delta_pixel_y * tan(self.visual_angle_y), self.image_shape[1])
        print ("-----Height is {}".format(self.human_position.human_dist / cos(angle_y) * sin(angle_y + self.angle_pitch)))
        # rospy.loginfo("Height is {}".format(self.human_position.human_dist / cos(angle_y) * sin(angle_y + self.angle_pitch)))
        self.human_position.human_angle = degrees(atan2(tan(angle_x) * cos(angle_y), cos(angle_y + self.angle_pitch)))
        self.human_position_pub.publish(self.human_position)
        
        


    # def getFaceDepth(self, depth_image_raw):
    #     if self.center_x == 0 or self.center_y == 0:
    #         rospy.loginfo("No face in sight....cannot get depth...")
    #         return -1
        
    #     try:
    #         depth_image = self.bridge.imgmsg_to_cv2(depth_image_raw, "bgr8")     
    #         # frame = np.array(cv_image, dtype=np.uint8)
    #     except CvBridgeError, e:
    #         print e

    #     num = 0
    #     self.image_shape = [depth_image_raw.width, depth_image_raw.height]
    #     #FIXME: 用numpy?
    #     begin_y = int(self.center_y - self.offset_y/2)
    #     end_y = int(self.center_y + self.offset_y/2)
    #     begin_x = int(self.center_x - self.offset_x/2)
    #     end_x = int(self.center_x + self.offset_x/2)
    #     print 

    #     for row in range(begin_y, end_y):
    #         for col in range(begin_x, end_x):
    #             if depth_image[row, col] != 0:
    #                 temp_depth = depth_image[row, col]
    #                 self.face_depth_mean = (self.face_depth_mean * num + temp_depth) / (num + 1)
    #                 num += 1
    #     rospy.loginfo("face_depth_mean is {}".format(self.face_depth_mean))

    #     getHumanPosition(self.center_x - self.image_shape[0] / 2, self.center_y - self.image_shape[1] / 2)

    #     self.human_position_pub.publish(self.human_position)

    # def getHumanPosition(self, delta_pixel_x, delta_pixel_y):
    #     angle_x = atan2(delta_pixel_x * tan(self.visual_angle_x) / self.image_shape[0])
    #     angle_y = atan2(delta_pixel_y * tan(self.visual_angle_y) / self.image_shape[1])
    #     rospy.loginfo("Height is {}".format(self.face_depth_mean / cos(angle_y) * sin(angle_y + self.angle_pitch) / 1000))

    #     self.human_position.human_angle = degrees(atan2(tan(angle_x) * cos(angle_y), cos(angle_y + self.angle_pitch)))
        # self.human_position.human_dist = self.face_depth_mean / cos(angle_y) * cos(angle_y + self.angle_pitch) / 1000

if __name__ == "__main__":
    rospy.init_node('human_position')
    # try:
    rospy.loginfo('start get human_info from face_info...')
    humanPos()
    rospy.spin()
    # except expression as identifier:
    #     pass