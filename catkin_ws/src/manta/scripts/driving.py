#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time, sys
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2,atan
from geometry_msgs.msg import Point,PoseWithCovarianceStamped,Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList ,GetTrafficLightStatus, GPSMessage
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_msgs.msg import String


class pure_pursuit :
    def __init__(self):
        rospy.init_node('driving', anonymous=True)
        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        # rospy.Subscriber("/local_path", Path, self.path_callback) #lattice_path #local_path #local_path_name
        rospy.Subscriber(local_path_name, Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic",ObjectStatusList,self.object_info_callback)
        rospy.Subscriber("/GetTrafficLightStatus",GetTrafficLightStatus,self.traffic_callback)
        rospy.Subscriber('/lane_stop', String, self.stop_callback)
        rospy.Subscriber("/is_find_object", String, self.obj_callback)
        rospy.Subscriber("/obj_idx", String, self.obj_idx_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        
        self.stopdis = 10
        self.station_x = 0
        self.station_y = 0
        self.ok_station = [0,1,0,0,0,0,0,0,0]
        self.objs = []
        self.objdis = 0.0
        
        self.is_look_forward_point = False

        self.current_postion = Point()
        self.ego = EgoVehicleStatus()

        self.vehicle_length = 2.72
        self.lfd = 8
        self.min_lfd= 5
        self.max_lfd= 30
        self.lfd_gain = 0.78
        self.target_velocity = 30
        self.traffic_msg = None
        
        self.pid = pidControl()
        self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain = 0.5, distance_gain = 1, time_gap = 0.8)
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)

        while True:
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                pass
                # rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.current_waypoint = self.get_current_waypoint([self.current_postion.x,self.current_postion.y],self.global_path) 
                if self.current_waypoint >= len(self.velocity_list):
                    self.target_velocity = 30
                else :
                    self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                
                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0
                self.target_velocity = self.adaptive_cruise_control.get_target_velocity(self.status_msg.velocity.x, self.target_velocity/3.6, 
                                                                                        self.traffic_msg,  self.status_msg, self.stopdis, self.ok_station, self.objs, self.objdis)

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6, self.traffic_msg, self.objs)
                
                if output > 5.0 and self.status_msg.velocity.x < 8.7 :
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True

    def object_info_callback(self,data):
        self.is_object_info = True
        self.object_data = data 

    def traffic_callback(self,msg):
        self.traffic_msg = msg
    
    def stop_callback(self, msg):
        self.stopdis=float(msg.data)

    def obj_callback(self, msg):
        self.objs = msg.data.split(',')

    def obj_idx_callback(self,msg):
        self.objdis=float(msg.data)/20
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1

        ego_pose_x = ego_status[0]
        ego_pose_y = ego_status[1]

        for i,pose in enumerate(global_path.poses):
            dx = ego_pose_x - pose.pose.position.x
            dy = ego_pose_y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint
    
    def calc_pure_pursuit(self,):
        self.lfd = (self.status_msg.velocity.x)*self.lfd_gain

        if self.lfd < self.min_lfd :
            self.lfd = self.min_lfd
        elif self.lfd > self.max_lfd :
            self.lfd = self.max_lfd

        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]
        trans_matrix = np.array([   [cos(self.vehicle_yaw)  ,-sin(self.vehicle_yaw) ,translation[0] ],
                                    [sin(self.vehicle_yaw)  ,cos(self.vehicle_yaw)  ,translation[1] ],
                                    [0                      ,0                      ,1              ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        dis = float('inf')
        # for num,i in enumerate(self.path.poses) :
        for i in self.path.poses:
            path_point = i.pose.position

            global_path_point = [path_point.x ,path_point.y , 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0]>0 :
                dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                if dis >= self.lfd :
                    self.is_look_forward_point = True
                    break
        theta = atan2(local_path_point[1],local_path_point[0])
        return atan((2*sin(theta)*self.vehicle_length)/dis)

class pidControl:
    def __init__(self):
        self.p_gain = 1.5
        self.i_gain = 3
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel, traffic, find_obj):
        error = target_vel - current_vel
        
        p_control = self.p_gain * error
        if traffic != None and not traffic.trafficLightStatus & 1 or traffic != None and not traffic.trafficLightStatus & 1 and find_obj is not ['0'] and self.i_control < 0:
            self.i_control = 0
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * ((self.prev_error - error) / self.controlTime)

        self.prev_error = error
        return p_control + self.i_control + d_control

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []
        Len = len(gloabl_path.poses)

        for i in range(0,point_num):
            out_vel_plan.append(5)
            # out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, Len - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)
            v_max = sqrt(r*9.8*self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(Len - point_num, Len-10):
            out_vel_plan.append(30)

        for i in range(Len - 10, Len):
            out_vel_plan.append(0)

        return out_vel_plan

class AdaptiveCruiseControl:
    #전역으로 처리
    def __init__(self, velocity_gain, distance_gain, time_gap):
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap

        self.sw = 0
        self.sw2 = 0
               

    def get_target_velocity(self, ego_vel, target_vel, traffic, car, stopdis, ok_ss, find_obj, obj_dis): 
        out_vel =  target_vel
        default_space = 8
        time_gap = self.time_gap
        v_gain = self.velocity_gain
        x_errgain = self.distance_gain
        #좌직 48 빨 1       
        nowIdx = [
            False,
            90 <= car.position.x <= 110 and 1144 <= car.position.y <= 1164,
            97 <= car.position.x <= 120 and 1255 <= car.position.y <= 1261,
            97 <= car.position.x <= 117 and 1357 <= car.position.y <= 1377,
            69 <= car.position.x <= 71  and 1578 <= car.position.y <= 1590,
            44 <= car.position.x <= 64  and 1137 <= car.position.y <= 1157,
            48 <= car.position.x <= 68  and 1903 <= car.position.y <= 1923,
            189<= car.position.x <= 209 and 1346 <= car.position.y <= 1366,
            38 <= car.position.x <= 58  and 1202 <= car.position.y <= 1222,
        ]  
        
        if traffic != None and traffic.trafficLightStatus & 1 and stopdis < 10 :
            print("ACC ON Red_traffic")
            if (int(ego_vel) == 0 or self.sw):
                self.sw = 1
                if stopdis >= 0:
                    out_vel = 1.4
                else:
                    out_vel = 0
            else :
                self.sw = 0
                stopdis += ego_vel*0.24 #마이너스값이 커지면 빨리 정거함 플러스면 늦게 정거함
                dis_safe = ego_vel * time_gap + default_space
                vel_rel= - ego_vel #ego_vel 차량의속도/3.6   
                acceleration = vel_rel * v_gain - x_errgain * (dis_safe - stopdis) #
                out_vel = ego_vel + acceleration #acceleration이 낮아져야 감속
        for i in range(1,9):
            if nowIdx[i] and ok_ss[i] ==0:
                print("ACC ON PARK",i)
                dis_safe = ego_vel * time_gap + default_space
                vel_rel = -ego_vel * (2 if i != 1 else 1)
                acceleration = vel_rel * v_gain - x_errgain * dis_safe
                out_vel = ego_vel + acceleration
                if int(ego_vel) <= 0 :
                    print("PARK",i)
                    time.sleep(3)
                    ok_ss[i]=1
                break  
        for obj in find_obj:
            print(obj_dis)
            if obj in ['1','2'] and obj_dis < 7:
                if (int(ego_vel) == 0 or self.sw2):
                    self.sw2 = 1
                    if obj_dis >= 0:
                        out_vel = 1.4
                    else:
                        out_vel = 0
                else :
                    print(obj_dis)
                    self.sw2 = 0
                    stopdis += ego_vel*0.24 #마이너스값이 커지면 빨리 정거함 플러스면 늦게 정거함
                    dis_safe = ego_vel * time_gap + default_space
                    vel_rel= - ego_vel #ego_vel 차량의속도/3.6   
                    acceleration = vel_rel * v_gain - x_errgain * (dis_safe - obj_dis) #
                    out_vel = ego_vel + acceleration 
                    break
                
        return out_vel * 3.6
     


if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
