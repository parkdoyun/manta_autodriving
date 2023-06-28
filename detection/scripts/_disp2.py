#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import sys
import math
import numpy as np

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


        self.display = None
        self.cluster_list = []

        self.dbscan = DBSCAN(eps=1, min_samples=25)

        global blue_color
        global yellow_color
        global green_color
        global red_color
        global white_color
        global black_color

        blue_color = (255, 0, 0)
        yellow_color = (0, 255, 255)
        green_color = (0, 255, 0)
        red_color = (0, 0, 255)
        white_color = (255, 255, 255)
        black_color = (0, 0, 0)

        self.obstacle = None

        self.img_bgrL = None
        self.img_bgrR = None

        self.cdstP = None

        self.target = 0
        self.targetL = 0
        self.targetR = 0
        self.queueL =[320]*5
        self.queueR = [320]*5

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            # 바탕화면 그리기
            self.obstacle = np.full((800,400,3), (255,255,255), dtype=np.uint8)
            
            if self.img_bgrL is not None and self.img_bgrR is not None:

                # self.LFA()

                img_roiL = self.img_bgrL[0:480, 0:480]
                img_roiR = self.img_bgrR[0:480, 160:640]

                addh = cv2.hconcat([img_roiL, img_roiR])

                cv2.imshow('addh', addh)
                cv2.waitKey(1)

            rate.sleep()

    def LFA(self):
        # cv2.line(self.img_bgrL, (350, 70), (350, 70), (0, 0, 255), 3)
        # cv2.line(self.img_bgrL, (400, 140), (400, 140), (0, 0, 255), 3)
        # cv2.line(self.img_bgrL, (450, 210), (450, 210), (0, 0, 255), 3)

        # 차선 BGR 범위 설정
        upper_white = np.array([255, 255, 255])
        lower_white = np.array([245, 245, 245])

        upper_yellow = np.array([10, 255, 255])
        lower_yellow = np.array([0, 245, 245])

        white_laneL = cv2.inRange(self.img_bgrL, lower_white, upper_white)
        yellow_laneL = cv2.inRange(self.img_bgrL, lower_yellow, upper_yellow)

        white_laneR = cv2.inRange(self.img_bgrR, lower_white, upper_white)
        yellow_laneR = cv2.inRange(self.img_bgrR, lower_yellow, upper_yellow)
        laneR = cv2.bitwise_or(white_laneR, yellow_laneR)

        img_LR = [self.img_bgrL, self.img_bgrR]

        # points = []
        # for i in range(1, 4):
        #     for j in range(1, 320):
        #         if yellow_laneL[70*i, 320-j]:
        #             points.append([70*i,320-j])
        #             break
        # print("\n\npointsY")
        # print(points)

        # if len(points) < 2:
        #     points = []
        #     for i in range(1, 4):
        #         for j in range(1, 320):
        #             if white_laneL[70*i, 320-j]:
        #                 points.append([70*i,320-j])
        #                 break

        #     print("\n\npointsL")
        #     print(points)
        #     if len(points) < 2:
        #         camera = 1
        #         points = []
        #         for i in range(1, 4):
        #             for j in range(0, 320):
        #                 if white_laneR[70*i, j]:
        #                     points.append([70*i,j])
        #                     break

        #         print("\n\npointsR")
        #         print(points)

        # alpha = 0
        # beta = 0
        # if len(points) >=2:
        #     x1 = points[0][1]
        #     y1 = points[0][0]
        #     x2 = points[1][1]
        #     y2 = points[1][0]

        #     cv2.line(img_LR[camera], (x1,y1), (x2,y2), (0, 0, 255), 3, cv2.LINE_AA)

        #     if x2-x1:
        #         alpha = float((y2-y1)/(x2-x1))
        #         if alpha < -1.73 or alpha > 1.73:
        #             beta = y1 - alpha*x1

        #         if alpha != 0:
        #             # self.target = (480-beta)/alpha
        #             self.target = ((600-beta)/alpha + self.target) / 2

        pointsL = []
        for i in range(1, 4):
            for j in range(1, 500):
                if yellow_laneL[70*i, 500-j]:
                    pointsL.append([70*i,500-j])
                    break

        if len(pointsL) < 2:
            pointsL = []
            for i in range(1, 4):
                for j in range(1, 500):
                    if white_laneL[70*i, 500-j]:
                        pointsL.append([70*i,500-j])
                        break

        if len(pointsL) >=2:
            x1_L = pointsL[0][1]
            y1_L = pointsL[0][0]
            x2_L = pointsL[1][1]
            y2_L = pointsL[1][0]

            if abs(x1_L-x2_L) < 100:
                cv2.line(img_LR[0], (x1_L,y1_L), (x2_L,y2_L), (0, 0, 255), 3, cv2.LINE_AA)
                self.targetL = (x1_L+x2_L)/2
                self.queueL.append(self.targetL)
                if len(self.queueL) > 5:
                    self.queueL.pop(0)

        pointsR = []
        for i in range(1, 4):
            for j in range(140, 640):
                # if white_laneR[70*i, j]:
                if laneR[70*i, j]:
                    pointsR.append([70*i,j])
                    break

        if len(pointsR) >=2:
            x1_R = pointsR[0][1]
            y1_R = pointsR[0][0]
            x2_R = pointsR[1][1]
            y2_R = pointsR[1][0]

            if abs(x1_R-x2_R) < 100:
                cv2.line(img_LR[1], (x1_R,y1_R), (x2_R,y2_R), (0, 0, 255), 3, cv2.LINE_AA)
                self.targetR = (x1_R+x2_R)/2
                self.queueR.append(self.targetR)
                if len(self.queueR) > 5:
                    self.queueR.pop(0)

        # self.drawLine(camera)
        self.draw()

        # 내 차 그리기
        self.obstacle = cv2.rectangle(self.obstacle, (165, 530), (235, 670), yellow_color, -1)

        cv2.imshow('img_bgrL', img_LR[0])
        cv2.imshow('img_bgrR', img_LR[1])

        cv2.imshow('self.obstacle', self.obstacle)
                
        cv2.waitKey(1)
        
    def draw(self):

        weights = [1,2,3,4,5]
        # meanL = np.mean(self.queueL)
        meanL = (sum(self.queueL[i]*weights[i] for i in range(len(weights)))) / sum(weights)

        # meanR = np.mean(self.queueR)
        meanR = (sum(self.queueR[i]*weights[i] for i in range(len(weights)))) / sum(weights)

        tL = int((meanL -360) * 35 / 200) + 165
        tR = int((meanR -280) * 35 / 200) + 235

        t_lane = (tL + tR)/2

        self.obstacle = cv2.line(self.obstacle, (t_lane-180,0), (t_lane-180,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t_lane-60,0), (t_lane-60,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t_lane+60,0), (t_lane+60,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t_lane+180,0), (t_lane+180,800), black_color, 2)
        
    def drawLine(self, camera):

        # 오른쪽 카메라
        if camera:
            t = int((self.target -280) * 35 / 200) + 235
    
            #차선 그리기
            self.obstacle = cv2.line(self.obstacle, (t,0), (t,800), black_color, 2)
            self.obstacle = cv2.line(self.obstacle, (t-130,0), (t-130,800), black_color, 2)
            self.obstacle = cv2.line(self.obstacle, (t+130,0), (t+130,800), black_color, 2)
            self.obstacle = cv2.line(self.obstacle, (t-260,0), (t-260,800), black_color, 2)

        # 왼쪽 카메라
        else:
            t = int((self.target -360) * 35 / 200) + 165
     
            #차선 그리기
            self.obstacle = cv2.line(self.obstacle, (t,0), (t,800), black_color, 2)
            self.obstacle = cv2.line(self.obstacle, (t-130,0), (t-130,800), black_color, 2)
            self.obstacle = cv2.line(self.obstacle, (t+130,0), (t+130,800), black_color, 2)
            self.obstacle = cv2.line(self.obstacle, (t+260,0), (t+260,800), black_color, 2)




    def LFA_hough(self):
        global camera
        
        camera = 1

        self.obstacle = np.full((800,400,3), (255,255,255), dtype=np.uint8)
      
        t = 135

        linesP = self.hough(self.img_bgrL)

        # 왼쪽 카메라
        if linesP is not None:
            max_beta = float(-2134567890)
            alpha = float(0)
            beta = float(0)
            for i in range(0, len(linesP)):     
                            
                l = linesP[i][0]
                # (x1, y1), (x2, x2)

                alpha_t = float((l[3]-l[1])/(l[2]-l[0]))

                if alpha_t < -1.73 or alpha_t > 1.73:
                    beta_t = l[1] - alpha_t*l[0]

                    cv2.line(self.cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)
                    
                    if beta_t > max_beta:
                        max_beta = beta_t
                        alpha = alpha_t
                        beta = beta_t

                if alpha != 0:
                    self.target_x = (480-beta)/alpha

            camera = 2

        #오른쪽 카메라
        # if linesP is None:
        else:
            linesP = self.hough(self.img_bgrR)  
              
            max_beta = float(-2134567890)
            alpha = float(0)
            beta = float(0)

            if linesP is not None:
                for i in range(0, len(linesP)):     
                                
                    l = linesP[i][0]
                    # (x1, y1), (x2, x2)
                    alpha_t = float((l[3]-l[1])/(l[2]-l[0]))
                    if alpha_t < -1.73 or alpha_t > 1.73:
                        beta_t = l[1] - alpha_t*l[0]

                        cv2.line(self.cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)
                        
                        if beta_t > max_beta:
                            max_beta = beta_t
                            alpha = alpha_t
                            beta = beta_t

                    if alpha != 0:
                        self.target_x = (480-beta)/alpha

                camera = 1


        

        if self.obstacle is not None:
            self.detect_obstacle()
            
        cv2.imshow("display", self.obstacle)

        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", self.cdstP)

        cv2.waitKey(1)


    def detect_obstacle(self):

        width = 35
        length = 70

        # 장애물 그리기
        for i in self.cluster_list:
            obj_x = int(i[0] * 20)
            obj_y = int(i[1] * 30)

            self.obstacle = cv2.rectangle(self.obstacle, (200-i[1]-width, 600-i[0]-length), (200-i[1]+width, 600-i[0]+length), red_color, -1) 

        # 내 차 그리기
        self.obstacle = cv2.rectangle(self.obstacle, (165, 530), (235, 670), yellow_color, -1)

    def hough(self, img):
        
        img_blur = cv2.blur(img, (9,9))

        lower_lane = np.array([0,245,245])
        upper_lane = np.array([255,255,255])
        img_lane = cv2.inRange(img_blur, lower_lane, upper_lane)
        
        # src = self.img_bgr

        src = img_lane

        dst = cv2.Canny(src, 50, 200, None, 3)

        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        self.cdstP = np.copy(cdst)


        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 150, None, minLineLength=150, maxLineGap=50)

        return linesP

        # cv2.imshow("Source", src)
        # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            #TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산
            
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])

            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
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
            
            for cluster in range(n_cluster):

                c_tmp = np.mean(pc_xy[db==cluster, :], axis=0)

                tmp_pose = Pose()
                tmp_pose.position.x = c_tmp.tolist()[0] + 1
                tmp_pose.position.y = c_tmp.tolist()[1]

                obj_x = (tmp_pose.position.x)*20
                obj_y = (tmp_pose.position.y)*30

                obj_x = int(obj_x)
                obj_y = int(obj_y)

                self.cluster_list.append([obj_x, obj_y])
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