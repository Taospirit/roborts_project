#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import torch
from torch.autograd import Variable
import torch.nn.functional as F
import torchvision.transforms as transforms

import torch.nn as nn
import torch.utils.data
import numpy as np
from opt import opt

from dataloader_ros import ImageLoader, DetectionLoader, DetectionProcessor, DataWriter, Mscoco
from yolo.util import write_results, dynamic_write_results
from SPPE.src.main_fast_inference import *

import os
import sys
from tqdm import tqdm
import time
from fn import getTime

from pPose_nms import pose_nms, write_json





from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError


args = opt
args.dataset = 'coco'
if not args.sp:
    torch.multiprocessing.set_start_method('forkserver', force=True)
    torch.multiprocessing.set_sharing_strategy('file_system')


class image_sub:
    def __init__(self):
        self.img_bgr8 = []
        rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback, queue_size=1)

    def imageCallback(self, img_data):
        try:
            self.img_bgr8 = CvBridge().imgmsg_to_cv2(img_data, "bgr8")
        except CvBridgeError:
            print (CvBridgeError)

    def getImage(self):
        return self.img_bgr8    


class alphaPoseDectector:
    def __init__(self):
        self.writer = 0
        self.image_list = []
        self.bridge = CvBridge()
        self.pose_dataset = Mscoco()
        self.image_sub = image_sub()

        self.start()
    
        self.pose_img_pub = rospy.Publisher("pose_detect_img", Image, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image, self.pubPoseImg, queue_size=1)

    def start(self):
        self.image_list.append(self.image_sub.getImage())
        # Load input images
        data_loader = ImageLoader(self.image_list, batchSize=args.detbatch, format='yolo', self.image_sub).start() # detbatch = 1

        # Load detection loader
        print('Loading YOLO model..')
        sys.stdout.flush()
        det_loader = DetectionLoader(data_loader, batchSize=args.detbatch).start()
        det_processor = DetectionProcessor(det_loader).start()

        pose_model = InferenNet_fast(4 * 1 + 1, self.pose_dataset)
        pose_model.cpu()
        pose_model.eval()

        batchSize = 80
        # Init data writer
        self.writer = DataWriter(args.save_video).start()

        data_len = data_loader.length() # 1
        im_names_desc = tqdm(range(data_len))

        batchSize = args.posebatch # 80
        for i in im_names_desc:
            with torch.no_grad():
                (inps, orig_img, im_name, boxes, scores, pt1, pt2) = det_processor.read() # 获取pose数据
                
                if boxes is None or boxes.nelement() == 0:
                    self.writer.save(None, None, None, None, None, orig_img, im_name.split('/')[-1])
                    continue
                
                # Pose Estimation
                datalen = inps.size(0)
                leftover = 0
                if (datalen) % batchSize:
                    leftover = 1
                num_batches = datalen // batchSize + leftover
                hm = []
                for j in range(num_batches):
                    inps_j = inps[j*batchSize:min((j +  1)*batchSize, datalen)].cpu()
                    hm_j = pose_model(inps_j)
                    hm.append(hm_j)
                hm = torch.cat(hm)

                hm = hm.cpu().data                
                self.writer.save(boxes, scores, hm, pt1, pt2, orig_img, im_name.split('/')[-1])

    def pubPoseImg(self, data):
        print ("pub pose_img")
        self.pose_img_pub.publish(self.bridge.cv2_to_imgmsg(self.writer.getImg(), "bgr8"))


    # def imageCallback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #         self.image_list.append(cv_image)     
    #     except CvBridgeError:
    #         print (CvBridgeError)

    #     while len(self.image_list) > 1:
    #         self.image_list.pop(0)

    #     # Load input images
    #     data_loader = ImageLoader(self.image_list, batchSize=args.detbatch, format='yolo').start() # detbatch = 1

    #     # Load detection loader
    #     print('Loading YOLO model..')
    #     sys.stdout.flush()
    #     det_loader = DetectionLoader(data_loader, batchSize=args.detbatch).start()
    #     det_processor = DetectionProcessor(det_loader).start()
        
    #     pose_model = InferenNet_fast(4 * 1 + 1, self.pose_dataset)
    #     pose_model.cpu()
    #     pose_model.eval()

    #     # Init data writer
    #     self.writer = DataWriter(args.save_video).start()

    #     data_len = data_loader.length() # 1
    #     im_names_desc = tqdm(range(data_len))

    #     batchSize = args.posebatch # 80
    #     for i in im_names_desc:
    #         with torch.no_grad():
    #             (inps, orig_img, im_name, boxes, scores, pt1, pt2) = det_processor.read() # 获取pose数据
                
    #             if boxes is None or boxes.nelement() == 0:
    #                 self.writer.save(None, None, None, None, None, orig_img, im_name.split('/')[-1])
    #                 continue
                
    #             # Pose Estimation
    #             datalen = inps.size(0)
    #             leftover = 0
    #             if (datalen) % batchSize:
    #                 leftover = 1
    #             num_batches = datalen // batchSize + leftover
    #             hm = []
    #             for j in range(num_batches):
    #                 inps_j = inps[j*batchSize:min((j +  1)*batchSize, datalen)].cpu()
    #                 hm_j = pose_model(inps_j)
    #                 hm.append(hm_j)
    #             hm = torch.cat(hm)

    #             hm = hm.cpu()
    #             self.writer.save(boxes, scores, hm, pt1, pt2, orig_img, im_name.split('/')[-1])

    #             self.pose_img_pub.publish(self.bridge.cv2_to_imgmsg(self.writer.getImg(), "bgr8"))

    #     print('===========================> Finish Model Running.')
    def out(self):
        print ('stop detect thread...')
        self.writer.stop()


if __name__ == "__main__":
    try:
        rospy.init_node('pose_detect_ros')
        print ("start to pose detect")
        pose = alphaPoseDectector()
        rospy.spin()
    except KeyboardInterrupt:
        pose.out()
        print ('stop')
