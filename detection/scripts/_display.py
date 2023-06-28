#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import sys
import math
import numpy as np

import os

from cv_bridge import CvBridgeError
from sklearn import linear_model

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN

class USERDisplay:
    def __init__(self):

        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callbackV)

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed_L", CompressedImage, self.callbackL)
        self.image_sub2 = rospy.Subscriber("/image_jpeg/compressed_R", CompressedImage, self.callbackR)

        self.cluster_pub = rospy.Publisher("clusters", PoseArray, queue_size=10)
        # self.d = [125,118,50]
        self.d = [125]


        self.display = None
        self.cluster_list = []
        self.cluster_list_BL = []
        self.cluster_list_BR = []

        self.obstacle_BL = False
        self.obstacle_BR = False

        self.dbscan = DBSCAN(eps=1.5, min_samples=25)

        global blue_color
        global yellow_color
        global green_color
        global red_color
        global white_color
        global black_color
        global orange_color

        blue_color = (255, 0, 0)
        yellow_color = (0, 255, 255)
        green_color = (0, 255, 0)
        red_color = (0, 0, 255)
        white_color = (255, 255, 255)
        black_color = (0, 0, 0)
        orange_color = (0, 165, 255)


        self.lane_color = 0
        
        self.obstacle = None
        self.mid_display = None
        self.display = None

        self.img_bgrL = None
        self.img_bgrR = None
        self.img_concat = None

        self.cdstP = None

        self.target = 0
        self.targetR = 0
        self.queue =[320]*5
        self.queueR = [320]*5

        self.logoL = cv2.imread('/home/ssafy/catkin_ws/src/ssafy_3/scripts/imgs/back_left.png')
        self.logoR = cv2.imread('/home/ssafy/catkin_ws/src/ssafy_3/scripts/imgs/back_right.png')

        self.logo_bus = cv2.imread('/home/ssafy/catkin_ws/src/ssafy_3/scripts/imgs/bus_back.png')
        self.logo_car_back = cv2.imread('/home/ssafy/catkin_ws/src/ssafy_3/scripts/imgs/car_back.png')
        self.logo_car_front = cv2.imread('/home/ssafy/catkin_ws/src/ssafy_3/scripts/imgs/car_front.png')

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            # 바탕화면 그리기
            self.obstacle = np.full((800,400,3), (255,255,255), dtype=np.uint8)

            self.display = np.full((900,1600,3), (255,255,255), dtype=np.uint8)
            
            if self.img_bgrL is not None and self.img_bgrR is not None:

                self.concat_camera()

                self.LFA()

                self.draw()

                self.detect_obstacle()

                # self.warp()

                self.detect_backward()

                cv2.imshow('obstacle', self.obstacle)
                # cv2.imshow('display', self.display)
                cv2.waitKey(1)

                rate.sleep()

    def warp(self):

        pts1 = np.float32([[0, 0], [0,800],[400,0], [400,800]])
        pts2 = np.float32([[400,200], [200,900], [1200,200], [1400,900]])

        rows, cols = self.display.shape[:2]

        mtrx = cv2.getPerspectiveTransform(pts1, pts2)

        self.display = cv2.warpPerspective(self.obstacle, mtrx, (cols, rows))

    def concat_camera(self):
        img_roiL = self.img_bgrL[0:480, 0:480]
        img_roiR = self.img_bgrR[0:480, 160:640]

        self.img_concat = cv2.hconcat([img_roiL, img_roiR])

        cv2.line(self.img_concat, (35, 50), (445, 50), (0, 0, 255), 3)

    def LFA(self):
        # cv2.line(self.img_bgrL, (350, 70), (350, 70), (0, 0, 255), 3)
        # cv2.line(self.img_bgrL, (400, 140), (400, 140), (0, 0, 255), 3)
        # cv2.line(self.img_bgrL, (450, 210), (450, 210), (0, 0, 255), 3)

        # 차선 BGR 범위 설정
        upper_white = np.array([255, 255, 255])
        lower_white = np.array([245, 245, 245])

        upper_yellow = np.array([10, 255, 255])
        lower_yellow = np.array([0, 245, 245])

        white_lane = cv2.inRange(self.img_concat, lower_white, upper_white)
        yellow_lane = cv2.inRange(self.img_concat, lower_yellow, upper_yellow)

        points = []

        for i in range(1, 4):
            max_x = 0
            for j in range(1, 480):
                if yellow_lane[5*i, j]:
                     max_x = j
                     self.lane_color = 1

            for j in range(1, 480):
                if yellow_lane[480 - 5*i, j]:
                     max_x = j
                     self.lane_color = 1
                    
            if max_x == 0:
                continue
            points.append([480 - 70*i, max_x])         

        if len(points) < 2:
            points = []
            for i in range(1, 4):
                for j in range(1, 960):
                    if white_lane[480 - 70*i, j]:
                        points.append([480 - 70*i, j])
                        self.lane_color = 2
                        break
        
        if len(points) >= 2:
            x1 = points[0][1]
            y1 = points[0][0]
            x2 = points[1][1]
            y2 = points[1][0]

            if abs(x1-x2) < 100:
                cv2.line(self.img_concat, (x1,y1), (x2,y2), (0, 0, 255), 3, cv2.LINE_AA)
                self.target = (x1+x2)/2
                self.queue.append(self.target)
                if len(self.queue) > 5:
                    self.queue.pop(0)
        
    def draw(self):

        # weights = [1,2,3,4,5]
        # # meanL = np.mean(self.queueL)
        # mean = (sum(self.queue[i]*weights[i] for i in range(len(weights)))) / sum(weights)

        # # meanR = np.mean(self.queueR)
        # meanR = (sum(self.queueR[i]*weights[i] for i in range(len(weights)))) / sum(weights)

        self.t_lane = int(((self.target-480) % 405) * 30 / 100) + 200
        
        # print("\n\ntlane\n")
        # print(self.target)

        if self.lane_color == 1:
            
            self.obstacle = cv2.line(self.obstacle, (self.t_lane-120,0), (self.t_lane-120,800), orange_color, 2)
        elif self.lane_color == 2:
            self.obstacle = cv2.line(self.obstacle, (self.t_lane-120,0), (self.t_lane-120,800), black_color, 2)

        self.obstacle = cv2.line(self.obstacle, (self.t_lane,0), (self.t_lane,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (self.t_lane-240,0), (self.t_lane-240,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (self.t_lane+120,0), (self.t_lane+120,800), black_color, 2)
        # self.obstacle = cv2.line(self.obstacle, (self.t_lane+240,0), (self.t_lane+240,800), black_color, 2)

    def detect_obstacle(self):
        t = self.t_lane

        width = 40
        length = 35

        # 장애물 그리기
        for i in self.cluster_list:
            obj_x = int(i[0] * 20)
            obj_y = int(i[1] * 30)

            obstacle_y = None
            
            # if self.lane_color == 1 and t - 240 < 200 - obj_y <= t - 120:
            #     obstacle_y = t - 180
            #     if 360 > obstacle_y - width > 0:
            #         self.draw_logo(self.logo_car_front, obstacle_y - width, 600 - obj_x - length)

            # elif self.lane_color == 2 and t - 240 < 200-obj_y <= t - 120:
            #     obstacle_y = t - 180
            #     if 360 > obstacle_y - width > 0:
            #         self.draw_logo(self.logo_car_back, obstacle_y - width, 600 - obj_x - length)

            # elif t - 120 < 200-obj_y <= t:
            #     obstacle_y = t - 60
            # elif t < 200-obj_y <= t + 120:
            #     obstacle_y = t + 60
            # elif t +120 < 200-obj_y <= t + 240:
            #     obstacle_y = t + 180

            # if obstacle_y is not None and 320 > obstacle_y - width > 0 and 600 - obj_x - length > 0:

            #     self.draw_logo(self.logo_car_back, obstacle_y - width, 600 - obj_x - length)

            #     # self.obstacle = cv2.rectangle(self.obstacle, (obstacle_y - width, 600 - obj_x-length), (obstacle_y+width, 600-obj_x+length), red_color, -1) 

            if t - 240 < 200 - obj_y <= t - 120:

                obstacle_y = t - 180        

            elif t - 120 < 200-obj_y <= t:

                obstacle_y = t - 60

            elif t < 200-obj_y <= t + 120:

                obstacle_y = t + 60

            elif t +120 < 200-obj_y <= t + 240:

                obstacle_y = t + 180

            if obstacle_y is not None and 0 < obstacle_y - width < 320 and 600 - obj_x - length > 0:

                if self.lane_color == 1 and t - 240 < 200 - obj_y <= t - 120:

                    self.draw_logo(self.logo_car_front, obstacle_y - width, 600 - obj_x - length)

                else:

                    self.draw_logo(self.logo_car_back, obstacle_y - width, 600 - obj_x - length)

                # self.obstacle = cv2.rectangle(self.obstacle, (obstacle_y - width, 600 - obj_x-length), (obstacle_y+width, 600-obj_x+length), red_color, -1) 

        # 내 차 그리기
        self.draw_logo(self.logo_bus, 200 - width, 600 - length)
        # self.obstacle = cv2.rectangle(self.obstacle, (200 - width, 600 - length), (200 + width, 600 + length), yellow_color, -1)

    def draw_logo(self, logo, y, x):
        h, w = logo.shape[:2]
        roi = self.obstacle[x:x+h, y:y+w]#배경이미지의 변경할(다음 로고 넣을) 영역
        mask = cv2.cvtColor(logo, cv2.COLOR_BGR2GRAY)#로고를 흑백처리
        #이미지 이진화 => 배경은 검정. 글자는 흰색
        # mask[mask[:]==0]=100
        mask[mask[:]==255]=0
        mask[mask[:]>0]=255
        mask_inv = cv2.bitwise_not(mask) #mask반전.  => 배경은 흰색. 글자는 검정
        daum = cv2.bitwise_and(logo, logo, mask=mask)#마스크와 로고 칼라이미지 and하면 글자만 추출됨
        back = cv2.bitwise_and(roi, roi, mask=mask_inv)#roi와 mask_inv와 and하면 roi에 글자모양만 검정색으로 됨
        dst = cv2.add(daum, back)#로고 글자와 글자모양이 뚤린 배경을 합침
        self.obstacle[x:x+h, y:y+w] = dst  #roi를 제자리에 넣음

    def detect_backward(self):
        if self.obstacle_BL:
            # print("BLBLBLBL\n")

            # self.draw_logo(self.logoL, 790, 610, self.obstacle)
            self.draw_logo(self.logoL, 75, 650)

        if self.obstacle_BR:
            # print("BRBRBRB\n")

            # self.draw_logo(self.logoR, 790, 890, self.obstacle)
            self.draw_logo(self.logoR, 225, 650)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            #TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산
            
            if point[3] not in self.d:
                continue
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])

            if 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))
            
        point_np = np.array(point_list, np.float32)

        return point_np

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

            self.obstacle_BL = False
            self.obstacle_BR = False
            
            for cluster in range(n_cluster):

                c_tmp = np.mean(pc_xy[db==cluster, :], axis=0)

                tmp_pose = Pose()
                tmp_pose.position.x = c_tmp.tolist()[0] + 1
                tmp_pose.position.y = c_tmp.tolist()[1]

                obj_x = (tmp_pose.position.x)
                obj_y = (tmp_pose.position.y)

                obj_x = int(obj_x)
                obj_y = int(obj_y)

                if obj_x > 0:

                    self.cluster_list.append([obj_x, obj_y])

                # elif obj_x < 0:
                #     if obj_y < 0:
                #         self.cluster_list_BL.append([obj_x, obj_y])
                #     else:
                #         self.cluster_list_BR.append([obj_x, obj_y])

                if obj_x < 0:
                    
                    if obj_y > 0.8:
                        self.obstacle_BL = True
                    elif obj_y < -0.8:
                        self.obstacle_BR = True
                
                cluster_msg.poses.append(tmp_pose)

        self.cluster_pub.publish(cluster_msg)


    def callbackL(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgrL = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
        except CvBridgeError as e:
            print(e)

    def callbackR(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgrR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
        except CvBridgeError as e:
            print(e)



if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    user_display = USERDisplay()

    rospy.spin()