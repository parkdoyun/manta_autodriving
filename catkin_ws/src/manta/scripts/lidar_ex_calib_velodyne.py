#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy, rospkg
import cv2, time, os, json
import numpy as np
from  math import cos, sin, pi
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN
from numpy.linalg import inv
from std_msgs.msg import String

rospack = rospkg.RosPack()
currentPath = rospack.get_path('manta')
with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
    sensor_params = json.load(fp)

parameters_cam = sensor_params["params_cam"]
parameters_lidar = sensor_params["params_lidar"]
objs = []
crop_pts = np.array(
                        [[
                            [int(parameters_cam["WIDTH"]  * 0.02), int(parameters_cam["HEIGHT"]  * 0.75)],
                            [int(parameters_cam["WIDTH"]  * 0.45), int(parameters_cam["HEIGHT"]  * 0.52)],
                            [int(parameters_cam["WIDTH"]  * 0.55), int(parameters_cam["HEIGHT"]  * 0.52)],
                            [int(parameters_cam["WIDTH"]  * 0.98), int(parameters_cam["HEIGHT"]  * 0.75)],
                            [int(parameters_cam["WIDTH"]  * 0.98), int(parameters_cam["HEIGHT"]  )],
                            [int(parameters_cam["WIDTH"]  * 0.02), int(parameters_cam["HEIGHT"]  )],
                        ]]
                    )
def getRotMat(RPY):
    cosR = cos(RPY[0])
    cosP = cos(RPY[1])
    cosY = cos(RPY[2])
    sinR = sin(RPY[0])
    sinP = sin(RPY[1])
    sinY = sin(RPY[2])

    rotRoll = np.array([1,0,0, 0,cosR,-sinR, 0,sinR,cosR]).reshape(3,3)
    rotPitch = np.array([cosP,0,sinP, 0,1,0, -sinP,0,cosP]).reshape(3,3)
    rotYaw = np.array([cosY,-sinY,0, sinY,cosY,0, 0,0,1]).reshape(3,3)
    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))
    return rotMat

def getSensorToVehicleMat(sensorRPY, sensorPosition):
    sensorRotationMat = getRotMat(sensorRPY)
    sensorTranslationMat = np.array([sensorPosition])
    Tr_sensor_to_vehicle = np.concatenate((sensorRotationMat,sensorTranslationMat.T),axis = 1)
    Tr_sensor_to_vehicle = np.insert(Tr_sensor_to_vehicle, 3, values=[0,0,0,1],axis = 0)
    
    return Tr_sensor_to_vehicle

def getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition):
    Tr_lidar_to_vehicle = getSensorToVehicleMat(lidarRPY, lidarPosition)
    Tr_cam_to_vehicle = getSensorToVehicleMat(camRPY, camPosition)
    Tr_vehicle_to_cam = inv(Tr_cam_to_vehicle)
    Tr_lidar_to_cam = Tr_vehicle_to_cam.dot(Tr_lidar_to_vehicle).round(6)
    
    return Tr_lidar_to_cam    

def getTransformMat(params_cam, params_lidar):
    lidarPositionOffset = np.array([0, 0, -0.25]) # VLP16 사용해야 함
    camPositionOffset = np.array([0.1085, 0, 0])  # Camera Offset  

    camPosition = np.array([params_cam.get(i) for i in (["X","Y","Z"])]) + camPositionOffset    
    camRPY = np.array([params_cam.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([-90*pi/180,0,-90*pi/180])
    lidarPosition = np.array([params_lidar.get(i) for i in (["X","Y","Z"])]) + lidarPositionOffset
    lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL","PITCH","YAW"])])    
    Tr_lidar_to_cam = getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition)
    return Tr_lidar_to_cam

def getCameraMat(params_cam):
    focalLength = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    principalX = params_cam["WIDTH"]/2
    principalY = params_cam["HEIGHT"]/2
    CameraMat = np.array([focalLength,0.,principalX,0,focalLength,principalY,0,0,1]).reshape(3,3)
    
    return CameraMat

class LiDARToCameraTransform:
    def __init__(self, params_cam, params_lidar):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.cluster_pub = rospy.Publisher("clusters", PoseArray, queue_size=10)
        self.pc_np = None
        self.img = None
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.CameraMat = getCameraMat(params_cam)
        self.dbscan = DBSCAN(eps=0.5, min_samples=5)
        self.d = [0,125,118,50]

    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg): 
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            cluster_msg = PoseArray()

        else:
            pc_xy = self.pc_np[:, :2]
            db = self.dbscan.fit_predict(pc_xy)
            n_cluster = np.max(db) + 1
            cluster_msg = PoseArray()

            for c in range(n_cluster):
                c_tmp = np.mean(pc_xy[db==c, :], axis=0)
                tmp_pose=Pose()
                tmp_pose.position.x = c_tmp.tolist()[0]
                tmp_pose.position.y = c_tmp.tolist()[1]
                cluster_msg.poses.append(tmp_pose)
        self.cluster_pub.publish(cluster_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        global objs
        l = []
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if point[0] > 0 and 1.50 > point[2] > -1.55 and dist < 20:
                for i in range(1,4):
                    if point[3] == self.d[i]:
                        point_list.append((point[0], point[1], point[2], point[3], dist, angle))
                        l.append(i)
                        break

        point_np = np.array(point_list, np.float32)
        objs = set(l)
        return point_np

    def transformLiDARToCamera(self, pc_lidar):
        pc_wrt_cam = self.TransformMat.dot(pc_lidar)
        pc_wrt_cam = np.delete(pc_wrt_cam, 3, axis=0)
        return pc_wrt_cam

    def transformCameraToImage(self, pc_camera):
        pc_proj_to_img = self.CameraMat.dot(pc_camera)
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[2,:]<0),axis=1)
        pc_proj_to_img /= pc_proj_to_img[2,:]
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[0,:]>self.width),axis=1)
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[1,:]>self.height),axis=1)
        return pc_proj_to_img
obj_idx_pub = None
def draw_pts_img(img, xi, yi):
    point_np = img.copy()

    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (0,255,0),-1)
    return point_np

def mask_roi(img):
    h = img.shape[0]
    w = img.shape[1]
    
    if len(img.shape)==3:
        c = img.shape[2]
        mask = np.zeros((h, w, c), dtype=np.uint8)
        mask_value = (255, 255, 255)

    else:
        mask = np.zeros((h, w), dtype=np.uint8)
        mask_value = (255)
    
    cv2.fillPoly(mask, crop_pts, mask_value)
    mask = cv2.bitwise_and(mask, img)
    return mask

def find_idx(img):
    img_mask = cv2.inRange(img, green, green)
    for i in range(479,0,-1):
        for j in range(0,320):
            if img_mask[i,320+j] or img_mask[i,320-j]:                
                return 360-i if i <= 340 else -20 #i 
    return 160
            

if __name__ == '__main__':
    rospy.init_node('ex_calib', anonymous=True)
    is_find_object_pub = rospy.Publisher("is_find_object", String, queue_size=10)
    obj_idx_pub = rospy.Publisher("obj_idx", String, queue_size=10)
    Transformer = LiDARToCameraTransform(parameters_cam, parameters_lidar)
    time.sleep(1)
    rate = rospy.Rate(10)
    green = np.array([0,255,0])
    dis = 160

    while not rospy.is_shutdown():
        mask = mask_roi(Transformer.img)
        mask2 = mask_roi(Transformer.img)
        if len(Transformer.pc_np) != 0:
            xyz_p = Transformer.pc_np[:, 0:3]
            xyz_p = np.insert(xyz_p,3,1,axis=1).T
            xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]<0),axis=1)

            xyz_c = Transformer.transformLiDARToCamera(xyz_p)

            xy_i = Transformer.transformCameraToImage(xyz_c)
            xy_i = xy_i.astype(np.int32)
            
            projectionImage = draw_pts_img(Transformer.img, xy_i[0,:], xy_i[1,:])
            projectionImage = draw_pts_img(mask, xy_i[0,:], xy_i[1,:])
            mask2 = mask_roi(projectionImage)
            dis = find_idx(mask2)
            
        
        # mask = mask_roi(Transformer.img)
        img_concat = np.concatenate([mask, mask2], axis=1)
        is_find_object_pub.publish( '0' if np.array_equal(mask, mask2) else ','.join(str(e) for e in objs) )
        obj_idx_pub.publish(str(dis))

        cv2.imshow("Image", img_concat)
        cv2.waitKey(1)
    cv2.destroyAllWindows()