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

from PIL import ImageFont, ImageDraw, Image

from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN


class IMGParser:
    def __init__(self, pkg_name = 'ssafy_3'):

        self.img_bgr = None
        
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callbackV)

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed_U", CompressedImage, self.callback)

        rate = rospy.Rate(5)

        self.d = [125,118,50]

        self.cluster_list = []

        self.logo_child1 = cv2.imread('/home/ssafy/catkin_ws/src/ssafy_3/scripts/imgs/child1.png')
        self.logo_child2 = cv2.imread('/home/ssafy/catkin_ws/src/ssafy_3/scripts/imgs/child2.png')

        self.min_distF = 0
        self.min_distB = 0

        self.dbscan = DBSCAN(eps=1.5, min_samples=15)

        while not rospy.is_shutdown():
            if self.img_bgr is not None:
                self.detectobstacle()

                rate.sleep()

    def detectobstacle(self):
        now = datetime.now()

        # obstacle_listF = [0, 0, 0, 0]
        # obstacle_listB = [0, 0, 0, 0]

        # lidar cluster
        
        min_distF = 2134567890
        min_distB = 2134567890
        for i in self.cluster_list:
            x = i[0]
            y = i[1]

            if x >= 4:
                min_distF = min(min_distF, i[2])
                
            elif x < -4:
                min_distB = min(min_distB, i[2])
        
        # if min_distF < self.min_distF:
        #     print("FRONT\n")
        #     if int(now.second) % 2:
        #         self.img_bgr = cv2.rectangle(self.img_bgr, (60,100), (420,200), (0,0,255), 5)
        #         cv2.putText(self.img_bgr, "FORWARD!", (90, 170), 0, 2, (0, 0, 255), 2)
 
        # if min_distB < self.min_distB:
        #     print("BACK\n")
        #     if int(now.second) % 2:
        #         self.img_bgr = cv2.rectangle(self.img_bgr, (60,440), (420,540), (0,0,255), 5)
        #         cv2.putText(self.img_bgr, "BACKWARD!", (90, 510), 0, 2, (0, 0, 255), 2)

        self.min_distF = min_distF
        self.min_distB = min_distB
        
                
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

        # # 한글 그리기
        # b,g,r,a = 255,255,255,0
        # fontpath = "fonts/sea.ttf"
        # font = ImageFont.truetype(fontpath, 40)
        # img_pil = Image.fromarray(self.img_bgr)
        # draw = ImageDraw.Draw(img_pil)
        # draw.text((60, 70),  '김형준', font=font, fill=(b,g,r))
        # self.img_bgr = np.array(img_pil)

        now = datetime.now()

        for cont in contours:
            # approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True) * 0.02, True)

            (x, y, w, h) = cv2.boundingRect(cont)
            pt1 = (x, y)
            pt2 = (x + w, y + h)
            # cv2.rectangle(self.img_bgr, pt1, pt2, (0, 255, 0), 2)
            # cv2.putText(self.img_bgr, label, (pt1[0], pt1[1]-3), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))

            if int(now.second) % 2:
                logo = self.logo_child1
            else:
                logo = self.logo_child2

            logo = self.logo_child1
            if int(now.second) % 2:
                if 240 <= (pt1[0]+pt2[0])/2 <= 480 and 120 <= (pt1[1]+pt2[1]) / 2 <= 520:
                    a = 300
                    b = 310
                    h, w = logo.shape[:2]
                    roi = self.img_bgr[a:a+h, b:b+w]#배경이미지의 변경할(다음 로고 넣을) 영역
                    mask = cv2.cvtColor(logo, cv2.COLOR_BGR2GRAY)#로고를 흑백처리
                    #이미지 이진화 => 배경은 검정. 글자는 흰색
                    mask[mask[:]==0]=100
                    mask[mask[:]>=200]=0
                    mask[mask[:]>0]=255
                    mask_inv = cv2.bitwise_not(mask) #mask반전.  => 배경은 흰색. 글자는 검정
                    daum = cv2.bitwise_and(logo, logo, mask=mask)#마스크와 로고 칼라이미지 and하면 글자만 추출됨
                    back = cv2.bitwise_and(roi, roi, mask=mask_inv)#roi와 mask_inv와 and하면 roi에 글자모양만 검정색으로 됨
                    dst = cv2.add(daum, back)#로고 글자와 글자모양이 뚤린 배경을 합침
                    self.img_bgr[a:a+h, b:b+w] = dst  #roi를 제자리에 넣음

                # if int(now.second) % 2:
                    # self.img_bgr = cv2.rectangle(self.img_bgr, (60,270), (420,370), (0,0,255), 5)
                    # cv2.putText(self.img_bgr, "WARNING!", (90, 340), 0, 2, (0, 0, 255), 2)
 

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        
        #self.detectLane("left", img_hsvL)

    def callbackV(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:

            cluster_msg = PoseArray()

        else:
            pc_xy = self.pc_np[:, :2]

            db = self.dbscan.fit_predict(pc_xy)

            n_cluster = np.max(db) + 1

            cluster_msg = PoseArray()

            self.cluster_list = []
            
            for cluster in range(n_cluster):

                c_tmp = np.mean(pc_xy[db==cluster, :], axis=0)

                tmp_pose = Pose()
                tmp_pose.position.x = c_tmp.tolist()[0] + 1
                tmp_pose.position.y = c_tmp.tolist()[1]
                dist = math.sqrt(tmp_pose.position.x**2 + tmp_pose.position.y**2)

                self.cluster_list.append([tmp_pose.position.x, tmp_pose.position.y, dist])
                cluster_msg.poses.append(tmp_pose)

        # self.cluster_pub.publish(cluster_msg)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            #TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산
            if point[3] in self.d:
                continue
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])

            if point[1] < 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))
            
        point_np = np.array(point_list, np.float32)

        return point_np

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()