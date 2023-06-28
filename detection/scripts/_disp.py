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

# lidar_velodyne_cluster 는 LiDAR의 Point들을 물체 단위로 구분하는 Clustering 예제입니다.
# PointCloud Data를 입력받아 DBSCAN Algorithm을 활용하여 Clustering을 수행합니다.
# 교육생분들은 DBSCAN의 Parameter를 조절하여 적절한 Clustering 결과를 얻어내야 합니다.

# 노드 실행 순서
# 1. DBSCAN Parameter 입력
# 2. 각 Cluster를 대표하는 위치 값 계산
# 3. PointCloud Data로부터 Distance, Angle 값 계산

class SCANCluster:
    def __init__(self):

        self.cluster_list = []

        # self.image_subL = rospy.Subscriber("/image_jpeg/compressed_L", CompressedImage, self.callbackL)
        # self.image_subR = rospy.Subscriber("/image_jpeg/compressed_R", CompressedImage, self.callbackR)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callbackV)

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed_L", CompressedImage, self.callbackL)
        self.image_sub2 = rospy.Subscriber("/image_jpeg/compressed_R", CompressedImage, self.callbackR)


        self.cluster_pub = rospy.Publisher("clusters", PoseArray, queue_size=10)

        rate = rospy.Rate(5)

        self.display = None
        self.cluster_list = []

        self.pc_np = None

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

        self.img_hsvL = None
        self.img_hsvR = None
        self.obstacle = None

        self.img_bgrL = None
        self.img_bgrR = None

        self.target_x = 0

        #TODO: (1) DBSCAN Parameter 입력
        '''
        # DBSCAN의 Parameter를 결정하는 영역입니다.
        # sklearn.cluster의 DBSCAN에 대해 조사하여 적절한 Parameter를 입력하기 바랍니다.

        self.dbscan = DBSCAN( , , ...)
        '''

        self.dbscan = DBSCAN(eps=1, min_samples=25)

        while not rospy.is_shutdown():

            # print("x, y === ", self.cluster_list)
            # print("\n\n")

            self.display = np.full((900,1600,3), (255,255,255), dtype=np.uint8)

            self.detect_obstacle()
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

        self.target_x = 0

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            if self.img_bgrL is not None and self.img_bgrR is not None:

                self.LFA()

            rate.sleep()


    def LFA(self):

        self.obstacle = np.full((800,400,3), (255,255,255), dtype=np.uint8)
      
        t = 135

        linesP = self.hough(self.img_bgrL)

        if linesP is None:
            linesP = self.hough(self.img_bgrR)  
              

        if linesP is not None:
            max_beta = float(-2134567890)
            alpha = float(0)
            beta = float(0)
            for i in range(0, len(linesP)):     
                            
                l = linesP[i][0]
                # (x1, y1), (x2, x2)
                cv2.line(self.cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

                alpha_t = float((l[3]-l[1])/(l[2]-l[0]))
                beta_t = l[1] - alpha_t*l[0]
                
                if beta_t > max_beta:
                    max_beta = beta_t
                    alpha = alpha_t
                    beta = beta_t

                if alpha != 0:
                    self.target_x = (480-beta)/alpha
            t = int((self.target_x -400) * 35 / 200) + 165
     
        #차선 그리기
        self.obstacle = cv2.line(self.obstacle, (t,0), (t,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t-130,0), (t-130,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t+130,0), (t+130,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t+260,0), (t+260,800), black_color, 2)


        if self.obstacle is not None:
            self.detect_obstacle()
            
        cv2.imshow("display", self.obstacle)
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
        
        img_blur = cv2.blur(img, (19,19))

        lower_lane = np.array([0,245,245])
        upper_lane = np.array([255,255,255])
        img_lane = cv2.inRange(img_blur, lower_lane, upper_lane)
        
        # src = self.img_bgr

        src = img_lane

        dst = cv2.Canny(src, 50, 200, None, 3)

        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        self.cdstP = np.copy(cdst)


        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 150, None, 100, 100)

        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", self.cdstP)
        cv2.waitKey(1)

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
            # if self.img_bgrL is not None and self.img_bgrR is not None:
                # self.houghR()
    

            self.detect_lane()
            # if self.img_hsvL is not None and self.img_hsvR is not None:
            #     self.LFA()

            #self.warp()
            
            # cv2.destroyAllWindows()


            rate.sleep()

        # self.display()

    def unite(self):

        self.display = np.zeros((900,1600,3), dtype=np.uint8)

        cv2.imshow("Main Display", self.display)
        cv2.waitKey(1)

    def callback(self, msg):    
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:

            cluster_msg = PoseArray()

        else:
            pc_xy = self.pc_np[:, :2]

            db = self.dbscan.fit_predict(pc_xy)

            n_cluster = np.max(db) + 1

            # print("pc_np =", self.pc_np)
            # print("\n\n")

            cluster_msg = PoseArray()

            self.cluster_list = []
            
            for cluster in range(n_cluster):
                #TODO: (2) 각 Cluster를 대표하는 위치 값 계산                
                '''
                # DBSCAN으로 Clustering 된 각 Cluster의 위치 값을 계산하는 영역입니다.
                # Cluster에 해당하는 Point들을 활용하여 Cluster를 대표할 수 있는 위치 값을 계산합니다.
                # 계산된 위치 값을 ROS geometry_msgs/Pose type으로 입력합니다.
                # Input : cluster
                # Output : cluster position x,y   

                tmp_pose=Pose()
                #tmp_pose.position.x = 
                #tmp_pose.position.y = 
                '''

                c_tmp = np.mean(pc_xy[db==cluster, :], axis=0)

                tmp_pose = Pose()
                tmp_pose.position.x = c_tmp.tolist()[0] + 1
                tmp_pose.position.y = c_tmp.tolist()[1]

                obj_x = (tmp_pose.position.x)*20
                obj_y = (tmp_pose.position.y)*30

                obj_x = int(obj_x)
                obj_y = int(obj_y)


                # print("x, y === ", obj_x, obj_y)

                # print("cluster ==", cluster)
                # print("tmp_pose ==", tmp_pose)
                # print("\n\n")

                # distance = np.sqrt(tmp_pose.position.x**2 + tmp_pose.position.y**2)
                # angle = np.arctan2(tmp_pose.position.x, tmp_pose.position.y)

                self.cluster_list.append([obj_x, obj_y])

                #불러올때 cluster_list[인덱스].get('id')
                # print("cluster ==", cluster)
                # print("tmp_pose ==", tmp_pose)
                # print("\n\n")

                cluster_msg.poses.append(tmp_pose)

            
            # print("cluster_liset == ", cluster_list)
            # print("\n\n")

        self.cluster_pub.publish(cluster_msg)

    def detect_obstacle(self):
        
        self.obstacle = np.full((800,400,3), (255,255,255), dtype=np.uint8)
        # obstacle = cv2.cvtColor(obstacle, cv2.COLOR_BGR2HSV)

        width = 35
        length = 70

        for i in self.cluster_list:
            obj_x = int(i[0] * 20)
            obj_y = int(i[1] * 30)

            # print("x, y === ", self.cluster_list)
            # print("\n\n")
            # self.display(obj_x, obj_y)
            self.obstacle = cv2.rectangle(self.obstacle, (200-i[1]-width, 600-i[0]-length), (200-i[1]+width, 600-i[0]+length), red_color, -1) 

        # cv2.imshow("obstacle", self.obstacle)
        # cv2.waitKey(1)
    
    def warp(self):

        pts1 = np.float32([[0, 0], [0,800],[400,0], [400,800]])
        pts2 = np.float32([[400,200], [200,900],[1200,200], [1400,900]])

        rows, cols = self.display.shape[:2]

        mtrx = cv2.getPerspectiveTransform(pts1, pts2)

        self.display = cv2.warpPerspective(self.obstacle, mtrx, (cols, rows))

        cv2.imshow("display", self.display)
        cv2.waitKey(1)
    
    def detect_lane(self):

        # t = int(self.target_x * 65 / 640) + 105
      
        t = int((self.target_x -400) * 35 / 200) + 165
     
        self.obstacle = cv2.line(self.obstacle, (t,0), (t,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t-130,0), (t-130,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t+130,0), (t+130,800), black_color, 2)
        self.obstacle = cv2.line(self.obstacle, (t+260,0), (t+260,800), black_color, 2)

        # 내 차 그리기
        self.obstacle = cv2.rectangle(self.obstacle, (165, 530), (235, 670), yellow_color, -1)

        cv2.imshow("ob", self.obstacle)
        cv2.waitKey(1)
        
    def LFA(self):
        # self.obstacle = cv2.line(self.obstacle, (5,0), (5,800), white_color, 2)
        # self.obstacle = cv2.line(self.obstacle, (135,0), (135,800), white_color, 2)
        # self.obstacle = cv2.line(self.obstacle, (265,0), (265,800), white_color, 2)
        # self.obstacle = cv2.line(self.obstacle, (395,0), (395,800), white_color, 2)

        lower_lane = np.array([15,45,170])
        upper_lane = np.array([45,255,255])
        img_laneL = cv2.inRange(self.img_hsvL, lower_lane, upper_lane)

        
        cv2.line(self.img_hsvL, (65,80),(65,400),(255,255,255),5)
        cv2.line(self.img_hsvL, (130,80),(130,400),(255,255,255),5)
        cv2.line(self.img_hsvL, (195,80),(195,400),(255,255,255),5)
        cv2.line(self.img_hsvL, (260,80),(260,400),(255,255,255),5)
        cv2.line(self.img_hsvL, (325,80),(325,400),(255,255,255),5)

        # 차선변경 확인용
        cv2.line(self.img_hsvL, (570,20),(570,150),(255,255,255),5)
        cv2.line(self.img_hsvL, (500,20),(500,150),(255,255,255),5)
        cv2.line(self.img_hsvL, (430,20),(430,150),(255,255,255),5)

        cv2.imshow("cameraL", self.img_hsvL)
        # cv2.imshow("cameraL", img_laneL)
        cv2.waitKey(1)

        img_laneR = cv2.inRange(self.img_hsvR, lower_lane, upper_lane)        

        cv2.line(self.img_hsvR, (315,80),(315,400),(255,255,255),5)
        cv2.line(self.img_hsvR, (380,80),(380,400),(255,255,255),5)
        cv2.line(self.img_hsvR, (445,80),(445,400),(255,255,255),5)
        cv2.line(self.img_hsvR, (510,80),(510,400),(255,255,255),5)
        cv2.line(self.img_hsvR, (575,80),(575,400),(255,255,255),5)

        # 차선변경 확인용
        cv2.line(self.img_hsvR, (70,20),(70,150),(255,255,255),5)
        cv2.line(self.img_hsvR, (140,20),(140,150),(255,255,255),5)
        cv2.line(self.img_hsvR, (210,20),(210,150),(255,255,255),5)

        cv2.imshow("cameraR", self.img_hsvR)
        cv2.waitKey(1)

    def hough(self):
        
        img_blur = cv2.blur(self.img_bgrL, (19,19))

        lower_lane = np.array([0,245,245])
        upper_lane = np.array([255,255,255])
        img_laneL = cv2.inRange(img_blur, lower_lane, upper_lane)
        
        # src = self.img_bgr

        src = img_laneL

        dst = cv2.Canny(src, 50, 200, None, 3)

        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        # lines = cv2.HoughLines(dst, 1, np.pi/180, 150, None, 0, 0)

        # if lines is not None:
        #     for i in range(0, len(lines)):
        #         rho = lines[i][0][0]
        #         theta = lines[i][0][1]
            # # if(theta * 180 /np.pi < 45 or theta * 180 /np.pi > 135):                    
            #     a = math.cos(theta)
            #     b = math.sin(theta)
            #     x0 = a * rho
            #     y0 = b * rho
            #     pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            #     pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            #     cv2.line(cdst, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            #     cv2.line(cdst, pt1, pt1, (255, 0, 0), 3, cv2.LINE_AA)
            #     cv2.line(cdst, pt2, pt2, (255, 0, 0), 3, cv2.LINE_AA)
            #     alpha = math.tan(theta)
            #     print("\n\nalpha\n")
            #     print(alpha)
            #     print("\n\nrho\n")
            #     print(rho)


        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 150, None, 100, 100)

        if linesP is not None:
            max_beta = float(-2134567890)
            alpha = float(0)
            beta = float(0)
            for i in range(0, len(linesP)):     
                           
                l = linesP[i][0]
                # (x1, y1), (x2, x2)
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

                alpha_t = float((l[3]-l[1])/(l[2]-l[0]))
                beta_t = l[1] - alpha_t*l[0]
                
                if beta_t > max_beta:
                    max_beta = beta_t
                    alpha = alpha_t
                    beta = beta_t


            # print("\n\nalpha\n")
            # print(alpha)

            # print("beta\n")
            # print(beta)

            if alpha != 0:
                self.target_x = (480-beta)/alpha
        
        else:
            img_flipped = cv2.flip(self.img_bgrR, 1)
            img_blur = cv2.blur(img_flipped, (19,19))

            lower_lane = np.array([0,245,245])
            upper_lane = np.array([255,255,255])
            img_laneL = cv2.inRange(img_blur, lower_lane, upper_lane)
            
            # src = self.img_bgr

            src = img_laneL

            dst = cv2.Canny(src, 50, 200, None, 3)

            cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
            cdstP = np.copy(cdst)

        # cv2.imshow("Source", src)
        # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

        cv2.waitKey(1)
    
    def houghR(self):
        
        img_blur = cv2.blur(self.img_bgrR, (19,19))

        lower_lane = np.array([0,245,245])
        upper_lane = np.array([255,255,255])
        img_laneL = cv2.inRange(img_blur, lower_lane, upper_lane)
        
        # src = self.img_bgr

        src = img_laneL

        dst = cv2.Canny(src, 50, 200, None, 3)

        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        # lines = cv2.HoughLines(dst, 1, np.pi/180, 150, None, 0, 0)

        # if lines is not None:
        #     for i in range(0, len(lines)):
        #         rho = lines[i][0][0]
        #         theta = lines[i][0][1]
            # # if(theta * 180 /np.pi < 45 or theta * 180 /np.pi > 135):                    
            #     a = math.cos(theta)
            #     b = math.sin(theta)
            #     x0 = a * rho
            #     y0 = b * rho
            #     pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            #     pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            #     cv2.line(cdst, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            #     cv2.line(cdst, pt1, pt1, (255, 0, 0), 3, cv2.LINE_AA)
            #     cv2.line(cdst, pt2, pt2, (255, 0, 0), 3, cv2.LINE_AA)
            #     alpha = math.tan(theta)
            #     print("\n\nalpha\n")
            #     print(alpha)
            #     print("\n\nrho\n")
            #     print(rho)


        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 150, None, 100, 100)

        if linesP is None:
            img_flipped = cv2.flip(self.img_bgrR, 1)
            img_blur = cv2.blur(img_flipped, (19,19))
            img_laneR = cv2.inRange(img_blur, lower_lane, upper_lane)
        
            src = img_laneR

            dst = cv2.Canny(src, 50, 200, None, 3)

            cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
            cdstP = np.copy(cdst)
            linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 150, None, 100, 100)

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

                    cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)
                    
                    if beta_t > max_beta:
                        max_beta = beta_t
                        alpha = alpha_t
                        beta = beta_t


            # print("\n\nalpha\n")
            # print(alpha)

            # print("beta\n")
            # print(beta)

            if alpha != 0:
                self.target_x = (480-beta)/alpha

        # cv2.imshow("Source", src)
        # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv2.imshow("Detected LinesR (in red) - Probabilistic Line Transform", cdstP)

        cv2.waitKey(1)

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
                #TODO: (2) 각 Cluster를 대표하는 위치 값 계산                
                '''
                # DBSCAN으로 Clustering 된 각 Cluster의 위치 값을 계산하는 영역입니다.
                # Cluster에 해당하는 Point들을 활용하여 Cluster를 대표할 수 있는 위치 값을 계산합니다.
                # 계산된 위치 값을 ROS geometry_msgs/Pose type으로 입력합니다.
                # Input : cluster
                # Output : cluster position x,y   

                tmp_pose=Pose()
                #tmp_pose.position.x = 
                #tmp_pose.position.y = 
                '''

                c_tmp = np.mean(pc_xy[db==cluster, :], axis=0)

                tmp_pose = Pose()
                tmp_pose.position.x = c_tmp.tolist()[0] + 1
                tmp_pose.position.y = c_tmp.tolist()[1]

                obj_x = (tmp_pose.position.x)*20
                obj_y = (tmp_pose.position.y)*30

                obj_x = int(obj_x)
                obj_y = int(obj_y)


                # print("x, y === ", obj_x, obj_y)

                # print("cluster ==", cluster)
                # print("tmp_pose ==", tmp_pose)
                # print("\n\n")

                # distance = np.sqrt(tmp_pose.position.x**2 + tmp_pose.position.y**2)
                # angle = np.arctan2(tmp_pose.position.x, tmp_pose.position.y)

                self.cluster_list.append([obj_x, obj_y])

                #불러올때 cluster_list[인덱스].get('id')
                # print("cluster ==", cluster)
                # print("tmp_pose ==", tmp_pose)
                # print("\n\n")

                cluster_msg.poses.append(tmp_pose)

            
            # print("cluster_liset == ", cluster_list)
            # print("\n\n")

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


    # def callbackL(self, msg):
    #     try:
    #         np_arr = np.fromstring(msg.data, np.uint8)
    #         img_bgrL = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #         self.img_hsvL = cv2.cvtColor(img_bgrL, cv2.COLOR_BGR2HSV)
    #     except CvBridgeError as e:
    #         print(e)

    # def callbackR(self, msg):
    #     try:
    #         np_arr = np.fromstring(msg.data, np.uint8)
    #         img_bgrR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #         self.img_hsvR = cv2.cvtColor(img_bgrR, cv2.COLOR_BGR2HSV)
    #     except CvBridgeError as e:
    #         print(e)

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



if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    scan_cluster = SCANCluster()

    rospy.spin()