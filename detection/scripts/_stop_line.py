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

# image_lane_fitting 은 Camera Image를 활용하여 차선 정보를 인지하는 예제입니다.
# Camera Image로 부터 차선의 위치에 해당하는 Pixel 좌표를 계산한 뒤,
# 좌우 차선 각각 RANSAC을 활용한 3차 곡선 근사를 수행합니다.

# 노드 실행 순서
# 1. CURVEFIT Parameter 입력
# 2. RANSAC Parameter 입력

class IMGParser:
    def __init__(self, pkg_name = 'ssafy_3'):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed_S", CompressedImage, self.callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)

        self.path_pub = rospy.Publisher('/lane_path', Path, queue_size=30)

        self.img_bgr = None
        self.img_lane = None
        self.edges = None 
        self.is_status = False

        self.lower_wlane = np.array([0,0,205])
        self.upper_wlane = np.array([30,60,255])

        self.lower_ylane = np.array([0,70,120])# ([0,60,100])
        self.upper_ylane = np.array([40,195,230])# ([40,175,255])

        self.crop_pts = np.array([[[0,480],[0,350],[280,200],[360,200],[640,350],[640,480]]])

        rospack = rospkg.RosPack()
        currentPath = rospack.get_path(pkg_name)
        
        with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
            sensor_params = json.load(fp)

        params_cam = sensor_params["params_cam"]

        bev_op = BEVTransform(params_cam=params_cam)
        #TODO: (1) CURVEFit Parameter 입력        
        # CURVEFit Class의 Parameter를 결정하는 영역입니다.
        # 하단의 CURVEFit Class에 대한 정보를 바탕으로 적절한 Parameter를 입력하기 바랍니다.

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.img_bgr is not None:

                img_crop = self.mask_roi(self.img_bgr)
                
                self.img_warp = bev_op.warp_bev_img(img_crop)

                # img_lane = self.binarize(img_warp)

                self.detectLane()

                # self.path_pub.publish(lane_path)

                cv2.waitKey(1)

                rate.sleep()

    
    def detectLane(self):

        lower_lane = np.array([0,0,140])
        upper_lane = np.array([65,65,255])

        img_lane = cv2.inRange(self.img_warp, lower_lane, upper_lane)

        # point_1 = self.img_bgr[300,320]
        # point_2 = self.img_bgr[350,320]


        for i in range(80, 160):
            for j in range(260, 380):
                if img_lane[i,j]:
                    print("slowdown")
                    break

        point_1 = img_lane[300,300]
        point_2 = img_lane[300,340]

        print("point 1", point_1)
        print("\n\n")
        print("point 2", point_2)
        print("\n\n")

        if point_1 == 255 or point_2 == 255:
            print("stop")
            print("\n\n")

        # print(self.img_hsvL[80, 480])

        cv2.line(self.img_warp, (260,80),(380,80),(255,0,0),5)

        cv2.line(self.img_warp, (260,160),(380,160),(255,0,0),5)


        cv2.line(self.img_warp, (300,300),(340,300),(255,0,0),5)

        # cv2.line(self.img_warp, (320,300),(320,300),(255,255,255),5)
        # cv2.line(self.img_warp, (320,350),(320,350),(255,255,255),5)

        # cv2.line(img_lane, (320,300),(320,300),255,5)
        # cv2.line(img_lane, (320,350),(320,350),255,5)

        cv2.imshow("camera", self.img_warp)
        cv2.imshow("camera1", img_lane)
        cv2.waitKey(1)

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.status_msg=msg    
        self.is_status = True

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

    def binarize(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_wlane = cv2.inRange(img_hsv, self.lower_wlane, self.upper_wlane)
        img_ylane = cv2.inRange(img_hsv, self.lower_ylane, self.upper_ylane)

        self.img_lane = cv2.bitwise_or(img_wlane, img_ylane)

        return self.img_lane

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape)==3:

            # num of channel = 3

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        else:
    
            # grayscale

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask = cv2.bitwise_and(mask, img)

        return mask



    def draw_lane_img(self, img, leftx, lefty, rightx, righty):
        '''
        place the lidar points into numpy arrays in order to make intensity map
        \n img : source image
        \n leftx, lefty, rightx, righty : curve fitting result 
        '''
        point_np = cv2.cvtColor(np.copy(img), cv2.COLOR_GRAY2BGR)

        #Left Lane
        for ctr in zip(leftx, lefty):
            point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

        #Right Lane
        for ctr in zip(rightx, righty):
            point_np = cv2.circle(point_np, ctr, 2, (0,0,255),-1)

        return point_np

class BEVTransform:
    def __init__(self, params_cam, xb=10.0, zb=10.0):
        self.xb = xb
        self.zb = zb

        self.theta = np.deg2rad(params_cam["PITCH"])
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.x = params_cam["X"]
        
        self.alpha_r = np.deg2rad(params_cam["FOV"]/2)

        self.fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        self.alpha_c = np.arctan2(params_cam["WIDTH"]/2, self.fc_y)

        self.fc_x = self.fc_y
            
        self.h = params_cam["Z"] + 0.34

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT_b2g = np.matmul(np.matmul(self.traslationMtx(xb, 0, zb),    self.rotationMtx(np.deg2rad(-90), 0, 0)),
                                                                            self.rotationMtx(0, 0, np.deg2rad(180)))

        self.proj_mtx = self.project2img_mtx(params_cam)

        self._build_tf(params_cam)


    def calc_Xv_Yu(self, U, V):
        Xv = self.h*(np.tan(self.theta)*(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r)-1)/\
            (-np.tan(self.theta)+(1-2*(V-1)/(self.m-1))*np.tan(self.alpha_r))

        Yu = (1-2*(U-1)/(self.n-1))*Xv*np.tan(self.alpha_c)

        return Xv, Yu


    def _build_tf(self, params_cam):
        v = np.array([params_cam["HEIGHT"]*0.5, params_cam["HEIGHT"]]).astype(np.float32)
        u = np.array([0, params_cam["WIDTH"]]).astype(np.float32)

        U, V = np.meshgrid(u, v)

        Xv, Yu = self.calc_Xv_Yu(U, V)

        xyz_g = np.concatenate([Xv.reshape([1,-1]) + params_cam["X"],
                                Yu.reshape([1,-1]),
                                np.zeros_like(Yu.reshape([1,-1])),
                                np.ones_like(Yu.reshape([1,-1]))], axis=0)
        
        xyz_bird = np.matmul(np.linalg.inv(self.RT_b2g), xyz_g)

        xyi = self.project_pts2img(xyz_bird)

        src_pts = np.concatenate([U.reshape([-1, 1]), V.reshape([-1, 1])], axis=1).astype(np.float32)
        dst_pts = xyi.astype(np.float32)

        self.perspective_tf = cv2.getPerspectiveTransform(src_pts, dst_pts)

        self.perspective_inv_tf = cv2.getPerspectiveTransform(dst_pts, src_pts)


    def warp_bev_img(self, img):
        img_warp = cv2.warpPerspective(img, self.perspective_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)
        
        return img_warp

    
    def warp_inv_img(self, img_warp):    
        img_f = cv2.warpPerspective(img_warp, self.perspective_inv_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)
        
        return img_f


    def recon_lane_pts(self, img):
        if cv2.countNonZero(img) != 0:
    
            UV_mark = cv2.findNonZero(img).reshape([-1,2])

            U, V = UV_mark[:, 0].reshape([-1,1]), UV_mark[:, 1].reshape([-1,1])
            
            Xv, Yu = self.calc_Xv_Yu(U, V)

            xyz_g = np.concatenate([Xv.reshape([1,-1]) + self.x,
                                Yu.reshape([1,-1]),
                                np.zeros_like(Yu.reshape([1,-1])),
                                np.ones_like(Yu.reshape([1,-1]))], axis=0)

            xyz_g = xyz_g[:, xyz_g[0,:]>=0]

        else:
            xyz_g = np.zeros((4, 10))

        return xyz_g


    def project_lane2img(self, x_pred, y_pred_l, y_pred_r):
        xyz_l_g = np.concatenate([x_pred.reshape([1,-1]),
                                  y_pred_l.reshape([1,-1]),
                                  np.zeros_like(y_pred_l.reshape([1,-1])),
                                  np.ones_like(y_pred_l.reshape([1,-1]))
                                  ], axis=0)

        xyz_r_g = np.concatenate([x_pred.reshape([1,-1]),
                                  y_pred_r.reshape([1,-1]),
                                  np.zeros_like(y_pred_r.reshape([1,-1])),
                                  np.ones_like(y_pred_r.reshape([1,-1]))
                                  ], axis=0)

        xyz_l_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_l_g)
        xyz_r_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_r_g)

        xyl = self.project_pts2img(xyz_l_b)
        xyr = self.project_pts2img(xyz_r_b)

        xyl = self.crop_pts(xyl)
        xyr = self.crop_pts(xyr)
        
        return xyl, xyr
        

    def project_pts2img(self, xyz_bird):
        xc, yc, zc = xyz_bird[0,:].reshape([1,-1]), xyz_bird[1,:].reshape([1,-1]), xyz_bird[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2,:].T
        
        return xyi

    def crop_pts(self, xyi):
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi


    def traslationMtx(self,x, y, z):     
        M = np.array([[1,         0,              0,               x],
                    [0,         1,              0,               y],
                    [0,         0,              1,               z],
                    [0,         0,              0,               1],
                    ])
        
        return M

    def project2img_mtx(self,params_cam):    
        '''
        project the lidar points to 2d plane
        \n xc, yc, zc : xyz components of lidar points w.r.t a camera coordinate
        \n params_cam : parameters from cameras 

        '''
        # focal lengths
        fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

        #the center of image
        cx = params_cam["WIDTH"]/2
        cy = params_cam["HEIGHT"]/2
        
        #transformation matrix from 3D to 2D
        R_f = np.array([[fc_x,  0,      cx],
                        [0,     fc_y,   cy]])

        return R_f

    def rotationMtx(self,yaw, pitch, roll):    
        R_x = np.array([[1,         0,              0,                0],
                        [0,         math.cos(roll), -math.sin(roll) , 0],
                        [0,         math.sin(roll), math.cos(roll)  , 0],
                        [0,         0,              0,               1],
                        ])
                        
        R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                        [0,                  1,      0               , 0],
                        [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                        [0,         0,              0,               1],
                        ])
        
        R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                        [math.sin(yaw),    math.cos(yaw),     0,    0],
                        [0,                0,                 1,    0],
                        [0,         0,              0,               1],
                        ])
                        
        R = np.matmul(R_x, np.matmul(R_y, R_z))
    
        return R

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 