import torch
from torch.autograd import Variable
import torch.nn.functional as F
import torchvision.transforms as transforms

import torch.nn as nn
import torch.utils.data
import numpy as np
from opt import opt

from dataloader_webcam import WebcamLoader, DetectionLoader, DetectionProcessor, DataWriter, crop_from_dets, Mscoco
from yolo.darknet import Darknet
from yolo.util import write_results, dynamic_write_results
from SPPE.src.main_fast_inference import *

from SPPE.src.utils.img import im_to_torch
import os
import sys
from tqdm import tqdm
import time
from fn import getTime
import cv2

from pPose_nms import write_json

args = opt
args.dataset = 'coco'


def loop():
    n = 0
    while True:
        yield n
        n += 1
def print_l(list):
    for i in list:
        print (type(i))
        print (len(i))

if __name__ == "__main__":
    webcam = args.webcam
    mode = args.mode
    if not os.path.exists(args.outputpath):
        os.mkdir(args.outputpath)

    # Load input video
    data_loader = WebcamLoader(webcam).start()
    (fourcc,fps,frameSize) = data_loader.videoinfo()

    # Load detection loader
    print('Loading YOLO model..')
    sys.stdout.flush()
    det_loader = DetectionLoader(data_loader, batchSize=args.detbatch).start()
    det_processor = DetectionProcessor(det_loader).start()
    
    # Load pose model
    pose_dataset = Mscoco()
    if args.fast_inference:
        pose_model = InferenNet_fast(4 * 1 + 1, pose_dataset)
    else:
        pose_model = InferenNet(4 * 1 + 1, pose_dataset)
    pose_model.cpu()
    pose_model.eval()

    # Data writer
    save_path = os.path.join(args.outputpath, 'AlphaPose_webcam'+webcam+'.avi')
    # 开始检测并绘制图像
    writer = DataWriter(args.save_video, save_path, cv2.VideoWriter_fourcc(*'XVID'), fps, frameSize).start()

    runtime_profile = {
        'dt': [],
        'pt': [],
        'pn': []
    }

    print('Starting webcam demo, press Ctrl + C to terminate...')
    sys.stdout.flush()
    im_names_desc =  tqdm(loop())
    batchSize = args.posebatch # 默认值: 80
    for i in im_names_desc:
        try:
            start_time = getTime()
            print ("------------------")
            with torch.no_grad():

                 # 人体数据
                (inps, orig_img, im_name, boxes, scores, pt1, pt2) = det_processor.read()
                info = (inps, orig_img, im_name, boxes, scores, pt1, pt2)
                print_l(info)

                if boxes is None or boxes.nelement() == 0:
                    # 存储进queue
                    writer.save(None, None, None, None, None, orig_img, im_name.split('/')[-1])
                    continue
                
                ###
                # ckpt_time, det_time = getTime(start_time) # 计算间隔时间
                # runtime_profile['dt'].append(det_time) # profile 计算时间间隔
                ###

                
                # Pose Estimation
                # 计算更新hm
                datalen = inps.size(0) # 使用inps数据
                leftover = 0
                if (datalen) % batchSize: # datalen 对80取余
                    leftover = 1
                num_batches = datalen // batchSize + leftover # datalen对80取整, 加上left_over
                hm = []
                for j in range(num_batches):
                    inps_j = inps[j*batchSize:min((j +  1)*batchSize, datalen)].cpu() # 使用inps数据
                    hm_j = pose_model(inps_j)
                    hm.append(hm_j)
                hm = torch.cat(hm)

                ###
                # ckpt_time, pose_time = getTime(ckpt_time)
                # runtime_profile['pt'].append(pose_time)
                ###

                hm = hm.cpu().data # 把hm的数据放在cpu上
                # 存储进queue
                writer.save(boxes, scores, hm, pt1, pt2, orig_img, im_name.split('/')[-1])

                ###
                # ckpt_time, post_time = getTime(ckpt_time)
                # runtime_profile['pn'].append(post_time)
                ###

            if args.profile:
                # TQDM
                im_names_desc.set_description(
                'det time: {dt:.3f} | pose time: {pt:.2f} | post processing: {pn:.4f}'.format(
                    dt=np.mean(runtime_profile['dt']), pt=np.mean(runtime_profile['pt']), pn=np.mean(runtime_profile['pn']))
                )
        except KeyboardInterrupt:
            break

    print(' ')
    print('===========================> Finish Model Running.')
    if (args.save_img or args.save_video) and not args.vis_fast:
        print('===========================> Rendering remaining images in the queue...')
        print('===========================> If this step takes too long, you can enable the --vis_fast flag to use fast rendering (real-time).')
    while(writer.running()):
        pass
    writer.stop()
    final_result = writer.results()
    write_json(final_result, args.outputpath)
