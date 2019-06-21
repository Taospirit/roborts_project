#!/usr/bin/env python
# -*- coding: utf-8 -*-


__author__ = 'Alex Wang'


import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

import cv2
import numpy as np

import sys, os
sys.path.append(os.path.join(os.getcwd(),'python/'))

import darknet as dn

from darknet_ros_msgs.msg import BoundingBox



def array_to_image(arr):
    arr = arr.transpose(2,0,1)
    c = arr.shape[0]
    h = arr.shape[1]
    w = arr.shape[2]
    arr = (arr/255.0).flatten()
    data = dn.c_array(dn.c_float, arr)
    im = dn.IMAGE(w,h,c,data)
    return im


class Detector(object):
    def __init__(self):
        rospy.init_node("yolo_detection",anonymous=1)

        # Darknet
        self.net = dn.load_net("/home/nvidia/hitsz_icra_2019/src/darknet_ros/darknet_ros/yolo_network_config/cfg/rm-yolov3-tiny.cfg", "/home/nvidia/hitsz_icra_2019/src/darknet_ros/darknet_ros/yolo_network_config/weights/rm-yolov3-tiny_170000.weights", 0)
        self.meta = dn.load_meta("/home/nvidia/hitsz_icra_2019/src/darknet_ros/darknet_ros/yolo_network_config/cfg/voc.data")

 # color image 
        self.sub = rospy.Subscriber("/industry_camera/image_raw", Image, self.image_cb)

        # operation related to OpenCV
        self.bridge = CvBridge()

        # publish detection results
        self.box = BoundingBox()
        self.detection_pub = rospy.Publisher('detection_result', BoundingBox, queue_size=1)



    # color image for detection and publish related messages
    def image_cb(self,color_msg):
        cv_image = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
        #image =  self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        cv_image = cv_image[32:992,:]
        cv_image = cv2.resize(cv_image,(640,480))
        im = array_to_image(cv_image)
        results = dn.detect2(self.net, self.meta, im)
        print results
        if len(results) != 0:
            results = results[0]
            self.box.xmin = results[2][0]
            self.box.ymin = results[2][1]
            self.box.xmax = results[2][2]
            self.box.ymax = results[2][3]
            cv2.putText(cv_image,'{}:{}'.format(results[0],results[1]),(results[2][0],results[2][1]),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),2)
            cv2.rectangle(cv_image,(results[2][0],results[2][1]),(results[2][2],results[2][3]),(0, 255, 0), 2)
            self.detection_pub.publish(self.box)
        else:
            rospy.logwarn('Do not detect any object in image!')
        #image = cv2.resize(image,(640,480))
        #cv2.imshow('Object',cv_image)
        #cv2.waitKey(1)


            
if __name__ == '__main__':
    detector = Detector()
    rospy.spin()



