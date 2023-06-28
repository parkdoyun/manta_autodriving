#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy, os, rospkg, cv2, json, math
import numpy as np
from cv_bridge import CvBridgeError

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class IMGParser:
    def __init__(self, pkg_name = 'manta'):
        rospy.Subscriber("/image_jpeg/compressed_S", CompressedImage, self.callback)
        self.stop_pub = rospy.Publisher('/lane_stop', String, queue_size=1)

        self.img_bgr = None
        self.img_lane = None

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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.img_bgr is not None:
                img_crop = self.mask_roi(self.img_bgr)
                self.img_warp = bev_op.warp_bev_img(img_crop)
                self.detectLane()
                cv2.waitKey(1)
            rate.sleep()

    def idx(self):
        cnt = [0]*6
        for i in range(340, 400):
            for j in range(300, 340):
                if self.img_lane[i,j]:
                    cnt[5] += 1
                if cnt[5] >= 30:
                    return '-1'
        for i in range(280, 340):
            for j in range(300, 340):
                if self.img_lane[i,j]:
                    cnt[0] += 1
                if cnt[0] >= 30:
                    return str((360-i)/20)
        for i in range(220, 280):
            for j in range(300, 340):
                if self.img_lane[i,j]:
                    cnt[1] += 1
                if cnt[1] >= 30:
                    return str((360-i)/22)
        for i in range(160, 220):
            for j in range(300, 340):
                if self.img_lane[i,j]:
                    cnt[2] += 1
                if cnt[2] >= 30:
                    return str((360-i)/24)
        return '10'
    
        
    def detectLane(self):
        lower_lane = np.array([0,0,140])
        upper_lane = np.array([65,65,255])

        self.img_lane = cv2.inRange(self.img_warp, lower_lane, upper_lane)
        
        self.stop_pub.publish(self.idx())

        cv2.line(self.img_lane, (260,200),(380,200),(255,0,255),5)
        cv2.line(self.img_lane, (260,340),(380,340),(255,0,0),5)
        cv2.line(self.img_lane, (260,400),(380,400),(255,0,0),5)
        cv2.imshow("stop lane", self.img_lane)
        cv2.waitKey(1)

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

    def mask_roi(self, img):
        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape)==3:
            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)
            mask_value = (255, 255, 255)
        else:
            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)
            mask_value = (255)
        cv2.fillPoly(mask, self.crop_pts, mask_value)
        mask = cv2.bitwise_and(mask, img)
        return mask

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
    rospy.init_node('lane_stop', anonymous=True)
    IMGParser()
    rospy.spin() 
