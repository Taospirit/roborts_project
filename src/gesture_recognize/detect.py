import cv2
import numpy as np
import copy
import math
""" import win32api """
""" import win32con """

cap_region_x_begin = 0.5
cap_region_y_end = 0.8
threshold = 60
blurValue = 41
bgSubThreshold = 25
learningRate = 0

isBgCaptured = 0
triggerSwitch = False


def printThreshold(thr):
    print("! Changed threshold to " + str(thr))


def removeBG(frame):
    fgmask = bgModel.apply(frame, learningRate=learningRate)
    kernel = np.ones((3, 3), np.uint8)
    fgmask = cv2.erode(fgmask, kernel, iterations=1)
    res = cv2.bitwise_and(frame, frame, mask=fgmask)
    return res

if __name__ == "__main__":
    camera = cv2.VideoCapture(0)
    camera.set(10, 10)
    cv2.namedWindow('trackbar')
    cv2.resizeWindow("trackbar", 640, 200)
    cv2.createTrackbar('threshold', 'trackbar', threshold, 100, printThreshold)
    i = 0
    while camera.isOpened():
        i = i + 1
        ret, frame = camera.read()
        threshold = cv2.getTrackbarPos('threshold', 'trackbar')
        frame = cv2.bilateralFilter(frame, 5, 50, 100)
        frame = cv2.flip(frame, 1)
        cv2.rectangle(frame, (int(cap_region_x_begin * frame.shape[1]), 0),(frame.shape[1], int(cap_region_y_end * frame.shape[0])), (0, 0, 255), 2)
        cv2.imshow('original', frame)

        if isBgCaptured == 1:
            img = removeBG(frame)
            img = img[0:int(cap_region_y_end * frame.shape[0]),int(cap_region_x_begin * frame.shape[1]):frame.shape[1]] 
            cv2.imshow('mask', img)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (blurValue, blurValue), 0)
            cv2.imshow('blur', blur)
            ret, thresh = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY)
            cv2.imshow('binary', thresh)

            thresh1 = copy.deepcopy(thresh)
            contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            length = len(contours)
            maxArea = -1
            if length > 0:
                for i in range(length):
                    temp = contours[i]
                    area = cv2.contourArea(temp)
                    if area > maxArea:
                        maxArea = area
                        ci = i

                res = contours[ci]
                hull = cv2.convexHull(res)
                drawing = np.zeros(img.shape, np.uint8)
                cv2.drawContours(drawing, [res], 0, (0, 255, 0), 2)
                cv2.drawContours(drawing, [hull], 0, (0, 0, 255), 3)

                moments = cv2.moments(res)
                center = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
                cv2.circle(drawing, center, 8, (0,0,255), -1)

                fingerRes = []
                max = 0; count = 0; notice = 0; cnt = 0
                for i in range(len(res)):
                    temp = res[i]
                    dist = (temp[0][0] -center[0])*(temp[0][0] -center[0]) + (temp[0][1] -center[1])*(temp[0][1] -center[1])
                    if dist > max:
                        max = dist
                        notice = i
                    if dist != max:
                        count = count + 1
                        if count > 40:
                            count = 0
                            max = 0
                            flag = False
                            if center[1] < res[notice][0][1]:
                                continue
                            for j in range(len(fingerRes)):
                                if abs(res[notice][0][0]-fingerRes[j][0]) < 20 :
                                    flag = True
                                    break
                            if flag :
                                continue
                            fingerRes.append(res[notice][0])
                            cv2.circle(drawing, tuple(res[notice][0]), 8 , (255, 0, 0), -1)
                            cv2.line(drawing, center, tuple(res[notice][0]), (255, 0, 0), 2)
                            cnt = cnt + 1

                cv2.imshow('output', drawing)
                Img_Name = "./" + str(i) + ".png"
                cv2.imwrite(Img_Name, drawing)
                print(cnt)
                if triggerSwitch is True:
                    if cnt >= 3:
                        print(cnt)
                        win32api.keybd_event(32, 0, 0, 0)
                        win32api.keybd_event(32, 0, win32con.KEYEVENTF_KEYUP, 0)

        k = cv2.waitKey(10)
        if k == 27:
            break
        elif k == ord('b'):
            bgModel = cv2.createBackgroundSubtractorMOG2(0, bgSubThreshold)
            isBgCaptured = 1
            print('!!!Background Captured!!!')
        elif k == ord('r'):
            bgModel = None
            triggerSwitch = False
            isBgCaptured = 0
            print('!!!Reset BackGround!!!')
        elif k == ord('n'):
            triggerSwitch = True
            print('!!!Trigger On!!!')