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
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        self.not_detect_count, self.not_detect_set = 0, 30

        self.human_position = HumanPosition()
        self.face_min_dist = 1300 # mm
        self.face_max_dist = 2800
        self.face_dist = 0

        rospy.Subscriber("face_position", FacePosition, self.getFacePosition, queue_size=1)
        rospy.Subscriber("supply_distance", SupplyDistance, self.faceDepthCallback, queue_size=1)
        rospy.Subscriber("robot_pose", Odometry, self.selfPoseCallback, queue_size=1)
        # rospy.Subscriber("/depth/image_rect_raw", Image, self.getFaceDepth, queue_size=1)
        self.human_position_pub = rospy.Publisher("human_position", HumanPosition, queue_size=1)

        self.point_pub = rospy.Publisher('position_show', PointStamped, queue_size=1)
        self.point_pub_ = PointStamped()
        self.point_pub_.header.frame_id = 'map'
        self.point_pub_.header.stamp = rospy.Time()
        self.point_pub_.point.x, self.point_pub_.point.y, self.point_pub_.point.z = 0, 0, 0
        self.robot_pose = {'x':0, 'y':0, 'theta':0}
        self.rs_theta = 0    

    def selfPoseCallback(self, data):
        self.robot_pose['x'] = data.pose.pose.position.x
        self.robot_pose['y'] = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        rad = euler_from_quaternion((qx,qy,qz,qw))
        self.robot_pose['theta'] = self.normalizeTheta(degrees(rad[2]))
        self.rs_theta = self.normalizeTheta(self.robot_pose['theta'] - 180)
        
    def getFacePosition(self, data):
        self.center_x = data.face_center_x
        self.center_y = data.face_center_y
        self.offset_x = data.face_offset_x
        self.offset_y = data.face_offset_y
        # print ("center is {:.2f} {:.2f}".format(self.center_x, self.center_y))

    def faceDepthCallback(self, data):
        print ("-----------------------------")
        if data.supply_distance == 0:
            print ("no face in sight for 1")
            self.not_detect_count += 1
        elif data.supply_distance < self.face_min_dist or data.supply_distance > self.face_max_dist:
            print ("no face in sight for 2")
            self.not_detect_count += 1
        else:
            self.face_dist = data.supply_distance # min meter
    
            delta_pixel_x = self.image_shape[0] / 2 - self.center_x
            delta_pixel_y = self.image_shape[1] / 2 - self.center_y
            angle_x = atan2(delta_pixel_x * tan(self.visual_angle_x), self.image_shape[0] / 2)
            angle_y = atan2(delta_pixel_y * tan(self.visual_angle_y), self.image_shape[1] / 2)
            print ("angle_x is {:.3f}, angle_y is {:.3f}".format(degrees(angle_x), degrees(angle_y)))

            self.human_position.human_angle = degrees(atan2(tan(angle_x) * cos(angle_y), cos(angle_y + self.angle_pitch)))
            human_dist_parallel = self.face_dist / cos(angle_y) * cos(angle_y + self.angle_pitch)
            self.human_position.human_dist = human_dist_parallel / cos(radians(self.human_position.human_angle))
            human_height = self.human_position.human_dist / cos(angle_y) * sin(angle_y + self.angle_pitch)

            print ("human_dist for parallel is {:.3f}".format(human_dist_parallel))
            print ("human_height is {}".format(human_height))

            self.not_detect_count = 0

        if self.not_detect_count > self.not_detect_set:
            self.human_position.human_dist, self.human_position.human_angle = 0, 0
            print ("*****clear info of human*****")
       
        print ('')
        # rospy.loginfo("Height is {}".format(self.human_position.human_dist / cos(angle_y) * sin(angle_y + self.angle_pitch)))
        self.human_position_pub.publish(self.human_position)

        human_theta = self.normalizeTheta(self.rs_theta + self.human_position.human_angle)
        self.point_pub_.point.x = self.human_position.human_dist / 1000 * cos(radians(human_theta)) + self.robot_pose['x'] + 0.15 * cos(radians(self.rs_theta))
        self.point_pub_.point.y = self.human_position.human_dist / 1000 * sin(radians(human_theta)) + self.robot_pose['y'] + 0.15 * sin(radians(self.rs_theta))
        print ("self_pose is {:.3f}, {:.3f}".format(self.robot_pose['x'], self.robot_pose['y']))
        self.point_pub.publish(self.point_pub_)

    def normalizeTheta(self, theta): # 将角度degree归一化为[-180, 180]
        return degrees(atan2(sin(radians(theta)), cos(radians(theta))))
        

if __name__ == "__main__":
    rospy.init_node('human_position')
    try:
        rospy.loginfo('start get human_info from face_info...')
        humanPos()
        rospy.spin()
    except e:
        print (e)