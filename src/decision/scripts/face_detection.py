#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import copy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from decision.msg import FacePosition

class faceDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_detected", Image, queue_size=1)
        self.face_position_pub = rospy.Publisher("face_position", FacePosition, queue_size=1)
        self.face_position = FacePosition()

        # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")

        # 使用级联表初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (50, 255, 50)

        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.x, self.y, self.w, self.h = 0, 0, 0, 0

    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # 创建灰度图像
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)

        # 尝试检测人脸
        faces_result = self.detect_face(grey_image)

        # 在opencv的窗口中框出所有人脸区域
        # show_img = copy.deepcopy(cv_image)
        # cv2.namedWindow('face_detect', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('face_detect', (int(640), int(480)))
        # cv2.moveWindow('face_detect', 100, 100)
        if len(faces_result)>0:
            for face in faces_result: 
                self.x, self.y, self.w, self.h = face
                # cv2.rectangle(show_img, (x, y), (x+w, y+h), self.color, 2)
                cv2.rectangle(cv_image, (self.x, self.y), (self.x+self.w, self.y+self.h), self.color, 2)
                print("-----Detected Face in image!{}".format(rospy.Time.now().secs))

                self.face_position.face_center_x = (self.x + self.w/2)
                self.face_position.face_center_y = (self.y + self.h/2)
                self.face_position.face_offset_x = self.w / 2
                self.face_position.face_offset_y = self.h / 2
                # cv2.circle(cv_image, (x + w/2, y + h/2), 3, (0, 0, 255),-1)
        else:
            print("-----NO face in sight{}".format(rospy.Time.now().secs))
        
        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        self.face_position_pub.publish(self.face_position)

    def detect_face(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
                                         
        # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
        
        return faces

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("face_detector")
        faceDetector()
        rospy.loginfo("Face detector is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
        cv2.destroyAllWindows()