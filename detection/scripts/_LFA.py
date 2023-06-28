#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import math
import random

from cv_bridge import CvBridgeError
from sklearn import linear_model

from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

class IMGParser:
    def __init__(self, pkg_name = 'ssafy_3'):

        self.img_hsvL = None
        self.img_hsvR = None

        self.image_subL = rospy.Subscriber("/image_jpeg/compressed_L", CompressedImage, self.callbackL)
        self.image_subR = rospy.Subscriber("/image_jpeg/compressed_R", CompressedImage, self.callbackR)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.img_hsvL is not None and self.img_hsvR is not None:
                self.detectLane()

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

    def detectLane(self):

        lower_lane = np.array([15,45,170])
        upper_lane = np.array([45,255,255])

        # lower_ylane = np.array([15,100,100])
        # upper_ylane = np.array([30,255,255])
        
        img_laneL = cv2.inRange(self.img_hsvL, lower_lane, upper_lane)

        
        point_UL = img_laneL[80,320]
        # print(point_yL)

        point_DL = img_laneL[400,320]

        if point_UL and point_DL:
            print("left")
        print("\n\n")

        # print(self.img_hsvL[80, 480])

        cv2.line(self.img_hsvL, (320,80),(320,80),(255,255,255),5)
        cv2.line(self.img_hsvL, (320,400),(320,400),(255,255,255),5)
        cv2.imshow("cameraL", self.img_hsvL)
        cv2.waitKey(1)
        
        
        img_laneR = cv2.inRange(self.img_hsvR, lower_lane, upper_lane)
        img_laneR = cv2.inRange(self.img_hsvR, lower_lane, upper_lane)
        
        point_UR = img_laneR[80,320]
        # print(point_wR)

        point_DR = img_laneR[400,320]
        # print(point_wR)

        if point_UR and point_DR:
            print("right")
        print("\n\n")

        cv2.line(self.img_hsvR, (320,80),(320,80),(255,255,255),5)
        cv2.line(self.img_hsvR, (320,400),(320,400),(255,255,255),5)
        cv2.imshow("cameraR", self.img_hsvR)
        cv2.waitKey(1)

    def callbackL(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgrL = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.img_hsvL = cv2.cvtColor(img_bgrL, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)
        
        #self.detectLane("left", img_hsvL)


    def callbackR(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgrR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.img_hsvR = cv2.cvtColor(img_bgrR, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)
        
        #self.detectLane("right", img_hsvR)

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()