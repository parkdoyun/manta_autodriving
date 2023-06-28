#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import math
import random
from datetime import datetime
from cv_bridge import CvBridgeError
from sklearn import linear_model

from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus


class IMGParser:
    def __init__(self, pkg_name = 'ssafy_3'):

        self.img_bgr = None

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed_B", CompressedImage, self.callback)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.img_bgr is not None:
                self.detectobstacle()

                rate.sleep()

    # def detectLane(self, camera, img_hsv):
    #     lower_wlane = np.array([0,20,180])
    #     upper_wlane = np.array([45,70,255])

    #     lower_ylane = np.array([15,100,100])
    #     upper_ylane = np.array([30,255,255])
        
    #     img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
    #     img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
        
    #     point_w = img_wlane[80,480]
    #     print(point_w)
    #     point_y = img_ylane[80,480]
    #     print(point_y)
    #     if point_w or point_y:
    #         print(camera)
    #     print("\n\n")

    #     cv2.line(img_hsv, (80,480),(80,480),(0,0,255),5)
    #     cv2.imshow(camera, img_hsv)
    #     cv2.waitKey(1)

    def detectobstacle(self):

        lower_red = np.array([50,50,245])
        upper_red = np.array([70,70,255])

        lower_blue = np.array([245,0,85])
        upper_blue = np.array([255,10,105])
        
        img_red = cv2.inRange(self.img_bgr, lower_red, upper_red)

        # ret, thr = cv2.threshold(img_red, 0, 255, cv2.THRESH_OTSU)

        # contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # for cont in contours:
        #     approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True) * 0.02, True)

        #     # vtc = len(approx)

        self.setLabel(img_red, 'car')

        #############################################

        img_blue = cv2.inRange(self.img_bgr, lower_blue, upper_blue)

        # ret, thr = cv2.threshold(img_blue, 0, 255, cv2.THRESH_OTSU)

        # _, contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # for cont in contours:
        #     approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True) * 0.02, True)

        #     # vtc = len(approx)

        self.setLabel(img_blue, 'man')


        # print("UP === ", self.img_bgr[100, 200])
        # print("DOWN === ", self.img_bgr[500, 200])
        # print("\n\n")

        # cv2.line(self.img_bgr, (200,100),(200,100),(255,255,255),5)
        # cv2.line(self.img_bgr, (200,500),(200,500),(255,255,255),5)

        cv2.imshow("blindspot", self.img_bgr)

        # cv2.imshow("red", img_red)
        # cv2.imshow("blue", img_blue)
        cv2.waitKey(1)

    def setLabel(self, img, label):
        # img = cv2.inRange(img, lower, upper)

        # img = cv2.GaussianBlur(img, (9,9), 0)
        img = cv2.blur(img, (19,19))

        ret, thr = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)

        _, contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #now = rospy.get_rostime()
        #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)


        for cont in contours:
            # approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True) * 0.02, True)

            (x, y, w, h) = cv2.boundingRect(cont)
            pt1 = (x, y)
            pt2 = (x + w, y + h)
            cv2.rectangle(self.img_bgr, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(self.img_bgr, label, (pt1[0], pt1[1]-3), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))

            now = datetime.now()

            if int(now.second) % 2:
                self.img_bgr = cv2.rectangle(self.img_bgr, (40,250), (360,350), (0,0,255), 5)
                cv2.putText(self.img_bgr, "WARNING!", (60, 320), 0, 2, (0, 0, 255), 2)

    # def box(self):


    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        
        #self.detectLane("left", img_hsvL)

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()