#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy

from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point, Point32, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage
from std_msgs.msg import String
import numpy as np
import tf
from pyproj import Proj
from collections import defaultdict

# FCM, MySQL
import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from firebase_admin import messaging
from datetime import datetime
from firebase_admin import db

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

'''
차량이 경로를 이탈했을 때, 다음 정류장까지의 경로를(A*) 새로 찾아서 따라간다.
전체 경로를 반환할때 2차원 리스트로 경로를 생성한다
[[0번에서1번 포인트들][1번에서 2번 포인트들][...][...]]
'''

class dijkstra_path_pub:
    def __init__(self):

        rospy.init_node('global_path', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.next_stop_pub = rospy.Publisher('/next_stop_location',String,queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_callback)

        # pure pursuit 위하여 데이터 수신
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        # GPS Message 수신 위하여 (버스의 현 위치 실시간 전송)
        rospy.Subscriber("/gps", GPSMessage, self.bus_pos_callback)
        rospy.Subscriber("driving/near_station", String, self.near_station_callback) # 하차 알림 보내기 위하여
        self.bus_pos_pub = rospy.Publisher('cpp/bus_position_log', String, queue_size=10)
        self.bus_log_pub = rospy.Publisher('cpp/bus_log', String, queue_size=10) # 하차 시 보낼 토픽
        self.bus_pose_cnt = 0

        # FCM 준비
        self.cred = credentials.Certificate('/home/lee/catkin_ws/key/firebase_key.json') # key file, 경로 바꾸세여
        self.app = firebase_admin.initialize_app(self.cred, {
            'databaseURL': 'https://reacttest-b8736-default-rtdb.firebaseio.com/'
        })

        # 해당하는 경로에서 실제로 버스가 이동한 point 값들
        self.real_bus = Path()
        self.real_bus.header.frame_id = '/map'

        # CPP로부터 응답
        rospy.Subscriber("cpp/station_fin", String, self.cpp_station_callback)

        # 현재 내가 쫓아가는 local_waypoint 알려주기
        self.current_waypoint = 0
        rospy.Subscriber('/local_point_pub', String, self.local_path_callback)

        # 상대적 위치를 알려준다
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)

        # 앞으로 갈 정류장 알려준다
        self.next_stop_pub = rospy.Publisher('/next_stop_pub',String , queue_size=10)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_odom = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 1
        self.lfd = 5

        # 유치원 번호 (일단 1, 호랑이 유치원이라 가정)
        self.kindergarten_num = 1
        self.in_out = 1  # 등원인지 하원인지, 일단 등원으로 가정

        rate = rospy.Rate(30)  # 30hz

        # # CPP 파일(file_listener.cpp)에서 MySQL에서 읽어온 뒤 파일에 써라
        self.kindergarten_info_pub = rospy.Publisher('cpp/get_station_info', String, queue_size=10)
        # self.kindergarten_info_pub.publish(str(self.kindergarten_num) + " " + str(self.in_out))
        # CPP로 항상 2번 보내야 함
        kinder_idx = 0
        while not rospy.is_shutdown():
            if kinder_idx == 8: break
            self.kindergarten_info_pub.publish(str(self.kindergarten_num) + " " + str(self.in_out))
            rospy.loginfo(str(self.kindergarten_num) + " " + str(self.in_out))
            kinder_idx += 1
            rate.sleep()
        
        self.cpp_fin_flag = 0
        
        # 파일에 써질 때까지 대기
        while True:
            if self.cpp_fin_flag == 1: break

        # TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.global_planner = Dijkstra(self.nodes, self.links)

        self.is_goal_pose = False
        self.is_init_pose = False
        #1 174 103.54695860884385 1160.0199978007004
        #2 699 109.88495861337287 1272.6199978012592
        #3 151 116.67995861923555 1367.948997802101
        #4 278 70.36895854462637 1577.2159978006966
        #5 266 50.44295853149379 1149.8159978012554
        #6 205 41.692958488478325 1912.9079977967776
        #7 248 199.13295874063624 1358.9039978012443
        #8 166 49.426958528289106 1207.8299977988936
        #9 158 94.18795859121019 1259.46899779886
        #10 282 133.732958642242 1411.3929977989756
        #11 226 143.8839586537797 1516.6519977990538

        self.way_point_node = []

         # 정류장 노드 번호 + 정류장 이름
        # 따로 dictionary 생성
        self.station_idx_name_dic = {}  # 정류장 노드 번호와 정류장 이름 딕셔너리 (나중에 FCM 전송 위하여)
        self.student_id_name_dic = {}  # 원생 ID와 원생 이름 딕셔너리
        # 정류장 ID와 node_idx 딕셔너리
        self.node_idx_id_dic = {1 : 'A119BS010174', 2 : 'A119BS010699', 3 : 'A119BS010151', 4 : 'A119BS010278', 5 : 'A119BS010266', 6 : 'A119BS010205', 
                                7 : 'A119BS010248', 8 : 'A119BS010166', 9 : 'A119BS010158', 10 : 'A119BS010282', 11 : 'A119BS010226'}
        
        # file에서 받은 거 읽기 (정류장의 노드 번호)
        with open("/home/lee/catkin_ws/route_info/route_station.txt", 'r') as file_data:
            line = None
            while True:
                line = file_data.readline().strip('\n')
                if line == '': break
                tmp_line_arr = line.split(' ')
                self.station_idx_name_dic[tmp_line_arr[0]] = tmp_line_arr[1]  # 딕셔너리에 추가
                if tmp_line_arr[0] not in self.way_point_node: self.way_point_node.append(tmp_line_arr[0]) # 중복 제거
        print(self.way_point_node) 
        print(self.station_idx_name_dic)
        
        self.student_station_dic = defaultdict(list)
        # file 읽기 (해당 정류장에서 내리는 원생들)
        with open("/home/lee/catkin_ws/route_info/route_student.txt", 'r') as file_data:
            line = None
            while True:
                line = file_data.readline().strip('\n')
                if line == '': break
                tmp_line_arr = line.split(' ')
                self.student_id_name_dic[tmp_line_arr[0]] = tmp_line_arr[2]  # 딕셔너리에 추가
                self.student_station_dic[tmp_line_arr[1]].append(tmp_line_arr[0])        
        print(self.student_station_dic)

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        # TSP 인자들
        self.VISITED_ALL = -1
        self.cache = []
        self.path = []

        self.node_order = []  # 경로 내 정류장의 x, y 값 (순서대로)

        # self.global_path_msg : 전체 경로의 모든 포인트들
        # self.global_node_path : 경로마다 각자 포인트들
        self.global_path_msg, self.global_node_path = self.calc_A_star_path_node()

        # 현재 몇번째 경로로 가고 있는지 기록한 변수
        self.current_node = 0

        # path make
        self.mysql_file_path = '/home/lee/catkin_ws/route_info/route.txt'  # mysql로 전송할 파일
        self.mysql_path_make()

        #
        # # MySql CPP 파일로 Publish 전송
        mysql_publisher = rospy.Publisher('file_listener', String, queue_size=10)
        #
        # # publish 두번 보내야 함
        mysql_idx = 0
        self.mysql_rate = rospy.Rate(5)
        self.mysql_path = "/home/lee/catkin_ws/route_info/route.txt"  # file path
        while not rospy.is_shutdown():
            if mysql_idx == 3: break  # 두번 보내면 실행됨
            rospy.loginfo("[MYSQL] route write <= " + self.mysql_path)
            mysql_publisher.publish(self.mysql_path)
            self.mysql_rate.sleep()
            mysql_idx += 1

        stop_len = len(self.way_point_node)
        self.next_stop = self.node_order[(self.current_node + 1) % stop_len]

        local_publisher = rospy.Publisher('/local_current_waypoint_update', String,
                                          queue_size=10)  # local의 self.current_waypoint 0으로 갱신하라

        while not rospy.is_shutdown():
            # pure pursuit
            if self.is_path == True and self.is_odom == True:
                
                #steering = self.calc_pure_pursuit()

                if self.is_look_forward_point:
                    #self.ctrl_cmd_msg.steering = steering
                    #self.ctrl_cmd_msg.velocity = 20.0
                    next_location = str(self.nodes[self.way_point_node[(self.current_node+1)%stop_len]].point[0]) + ',' + str(self.nodes[self.way_point_node[(self.current_node + 1) % stop_len]].point[1])
                    # 현재 local_route에서 내가 쫒아가는 포인트와의 거리 (local_route의 첫번째 포인트)
                    tmp_dis = sqrt(pow(self.current_postion.x - self.global_node_path[self.current_node].poses[
                        self.current_waypoint+1].pose.position.x, 2) + pow(
                        self.current_postion.y - self.global_node_path[self.current_node].poses[
                            self.current_waypoint+1].pose.position.y, 2))

                    if tmp_dis >= 100:
                        # 가장 가까운 노드 찾기
                        near_node_idx = self.init_node(self.current_postion.x, self.current_postion.y)
                        # 다음 정류장 노드까지의 A*star로 경로 찾고
                        tmp_result, tmp_path = self.global_planner.A_star(self.nodes[near_node_idx], self.nodes[
                            self.way_point_node[(self.current_node + 1) % stop_len]], near_node_idx,
                                                                          self.way_point_node[
                                                                              (self.current_node + 1) % stop_len])

                        # global_path로 보낼 path 만들기
                        tmp_new_path = Path()
                        tmp_new_path.header.frame_id = '/map'  # 이거 안 쓰면 rviz 화면에 안 찍힘frame_id = '/map' # 이거 안 쓰면 rviz 화면에 안 찍힘
                        # local로 보내는 경로에는 새롭게 계산한 경로만 가야 한다
                        tmp_new_back_path = Path()
                        tmp_new_back_path.header.frame_id = '/map'  # 이거 안 쓰면 rviz 화면에 안 찍힘frame_id = '/map' # 이거 안 쓰면 rviz 화면에 안 찍힘

                        # 지금까지 버스가 지나온 point 값들 path 추가
                        for idx, i in enumerate(self.real_bus.poses):
                            tmp_point = PoseStamped()
                            x = i.pose.position.x
                            y = i.pose.position.y
                            tmp_point.pose.position.x = float(x)
                            tmp_point.pose.position.y = float(y)
                            tmp_point.pose.orientation.w = 1
                            tmp_new_path.poses.append(tmp_point)

                        # 현재에서 다음 정류장까지 path 추가
                        for idx in tmp_path['point_path']:
                            tmp_point = PoseStamped()
                            tmp_point.pose.position.x = float(idx[0])
                            tmp_point.pose.position.y = float(idx[1])
                            tmp_point.pose.orientation.w = 1
                            tmp_new_path.poses.append(tmp_point)
                            tmp_new_back_path.poses.append(tmp_point)

                        # 전체 경로를 갱신
                        self.global_node_path[self.current_node] = tmp_new_path

                        # 다시 global path로 publish
                        # local_publisher.publish("1")  # local_path_pub의 current_waypoint 갱신하라
                        self.path_make()
                        rate.sleep()

                else:
                    # 가려는 정류장에 근접했다면
                    tmp_dis = sqrt(pow(self.current_postion.x - self.next_stop[0], 2) + pow(
                        self.current_postion.y - self.next_stop[1], 2))
                    # print("TMP_DIS => " + str(tmp_dis))
                    # print(self.next_stop)
                    pub_stop = str(self.next_stop[0]) + "," + str(self.next_stop[1])
                    self.next_stop_pub.publish(pub_stop)
                    # self.next_stop_pub.publish(next_location)
                    if tmp_dis < 5:  # 다음 경로 publish 하고 가리키는 현재 노드 갱신
                        # local_publisher.publish("1")  # 갱신하라

                        # 만일 목적한 도착지에 도착하게 되면 지금까지 온 경로를 현재 경로에 저장하고
                        # 버스의 경로 초기화
                        self.global_node_path[self.current_node] = self.real_bus
                        self.current_node = (self.current_node + 1) % stop_len
                        self.real_bus = Path()
                        self.real_bus.header.frame_id = '/map'
                        self.path_make()
                        print(self.current_node)
                        self.next_stop = self.node_order[(self.current_node + 1) % stop_len]
                        rate.sleep()  # 바로 다음 경로로 넘어가는 것 방지
                        rate.sleep()
                        # #
                        # # # publish 두번 보내야 함
                        # mysql_idx = 0
                        # while not rospy.is_shutdown():
                        #     if mysql_idx == 10: break  # 두번 보내면 실행됨
                        #     rospy.loginfo("[MYSQL] route write <= " + self.mysql_path)
                        #     mysql_publisher.publish(self.mysql_path)
                        #     self.mysql_rate.sleep()
                        #     mysql_idx += 1
                   
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.pure_path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = tf.transformations.euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y
        tmp_bus_point = PoseStamped()
        tmp_bus_point.pose.position.x = float(self.current_postion.x)
        tmp_bus_point.pose.position.y = float(self.current_postion.y)
        tmp_bus_point.pose.orientation.w = 1
        self.real_bus.poses.append(tmp_bus_point)

    def status_callback(self, msg):  ## Vehicle Status Subscriber
        self.is_status = True
        self.status_msg = msg

    def local_path_callback(self, msg):
        self.current_waypoint = int(msg.data)

    def cpp_station_callback(self, msg):
        self.cpp_fin_flag = 1

    # =========================================추가 코드==============================================
    # 보내지는지 필수로 확인해야 함 (DB, FCM)
    def bus_pos_callback(self, msg): # 버스 현 위치 전송
        self.bus_pose_cnt+=1
        if self.bus_pose_cnt < 1400:
            
            return
        self.bus_pose_cnt=0
        
        # cpp로 현 위경도 전송
        lat = msg.latitude # (morai_msgs/GPSMessage)
        lon = msg.longitude
        
        rospy.loginfo("bus_pos_callback => " + str(lat) + " " + str(lon))
        for i in range(3): self.bus_pos_pub.publish(str(lat) + " " + str(lon))
        # 주기적으로 실행받아야 함

    def near_station_callback(self, msg): # 하차 시 DB, FCM
        # msg : 정류장 ID
        # node_idx로 변환하여 DB 전송 및 FCM 푸시 필요
        station_ID = int(msg.data) # 정류장 번호
        in_out_str = "등원"
        if self.in_out == 0: in_out_str = "하원" # 등원/하원 상태 변경
        for i in self.student_station_dic[self.node_idx_id_dic[station_ID]]: # i : 원생들 id
            self.bus_log_pub.publish(i + " " + str(self.in_out) + " " + msg.data) # 하차 신호 보내기
            rospy.loginfo("near_station_callback => " + i + " " + str(self.in_out) + " " + msg.data)
            # FCM 전송
            dir = db.reference(i + '/token')
            fcm_token = dir.get()
            rospy.loginfo("near_station_callback FCMTOKEN[" + fcm_token + "]")
            now = datetime.now()

            message = messaging.Message(
                notification=messaging.Notification(
                    title=in_out_str,
                    body=self.station_idx_name_dic[self.node_idx_id_dic[station_ID]] + " " + now.strftime('%Y-%m-%d %H:%M:%S') # 정류장명 + 시간
                ),
                data={
                    'name': self.student_id_name_dic[i], # 원생 이름
                },
                token=fcm_token,
            )
            response = messaging.send(message)
            print(response)


    def calc_pure_pursuit(self, ):
        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

        det_trans_matrix = np.linalg.inv(trans_matrix)
        # 모든 노드를 확인한다. local에 있는
        for num, i in enumerate(self.pure_path.poses):
            # path_point에 x, y, z가 들어있다. node 의 위치 / local의 노드
            path_point = i.pose.position
            # global_path_point = local 노드이 정보
            global_path_point = [path_point.x, path_point.y, 1]
            # 변환
            local_path_point = det_trans_matrix.dot(global_path_point)
            # local_path_point[0] = 변환된 x의 좌표
            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        # TODO: (3) Steering 각도 계산
        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * self.vehicle_length * sin(theta)),
                         sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2)))

        return steering

    def mysql_path_make(self):
        self.mysql_f = open(self.mysql_file_path, 'w')

        global_node_path_msg = Path()
        global_node_path_msg.header.frame_id = '/map'

        e2u_zone = 52
        e_offset = 302459.942  # east offset
        n_offset = 4122635.537  # north offset
        e2u_conv = Proj(proj='utm', zone=e2u_zone, ellps='WGS84', preserve_units=False)

        idx1, idx2 = 1, 0
        for now in range(len(self.global_node_path)):
            for idx, i in enumerate(self.global_node_path[now].poses):
                idx2 += 1
                if idx2 % 30 == 0:
                    x = i.pose.position.x
                    y = i.pose.position.y
                    z = -0.511265
                    data = '{0}\t{1}\t{2}\n'.format(x, y, z)

                    # WGS84로 변환하여 넣기
                    utmx = x + e_offset
                    utmy = y + n_offset

                    # 변환
                    lon, lat = e2u_conv(utmx, utmy, inverse=True)

                    # 쓰기
                    self.mysql_f.write(str(idx1) + " " + str(lon) + " " + str(lat) + "\n")
                    idx1 += 1
        for now in range(len(self.global_node_path)):
            for idx, i in enumerate(self.global_node_path[now].poses):
                x = i.pose.position.x
                y = i.pose.position.y

                tmp_point = PoseStamped()
                tmp_point.pose.position.x = float(x)
                tmp_point.pose.position.y = float(y)
                tmp_point.pose.orientation.w = 1
                global_node_path_msg.poses.append(tmp_point)

        self.global_path_pub.publish(global_node_path_msg)
        self.mysql_f.close()
    
    def path_make(self):
        global_node_path_msg = Path()
        global_node_path_msg.header.frame_id = '/map'

        e2u_zone = 52
        e_offset = 302459.942  # east offset
        n_offset = 4122635.537  # north offset
        e2u_conv = Proj(proj='utm', zone=e2u_zone, ellps='WGS84', preserve_units=False)

        idx1, idx2 = 1, 0
        for now in range(len(self.global_node_path)):
            for idx, i in enumerate(self.global_node_path[now].poses):
                idx2 += 1
                if idx2 % 30 == 0:
                    x = i.pose.position.x
                    y = i.pose.position.y
                    z = -0.511265
                    data = '{0}\t{1}\t{2}\n'.format(x, y, z)

                    # WGS84로 변환하여 넣기
                    utmx = x + e_offset
                    utmy = y + n_offset

                    # 변환
                    lon, lat = e2u_conv(utmx, utmy, inverse=True)

                    # 쓰기
                    idx1 += 1
        for now in range(len(self.global_node_path)):
            for idx, i in enumerate(self.global_node_path[now].poses):
                x = i.pose.position.x
                y = i.pose.position.y

                tmp_point = PoseStamped()
                tmp_point.pose.position.x = float(x)
                tmp_point.pose.position.y = float(y)
                tmp_point.pose.orientation.w = 1
                global_node_path_msg.poses.append(tmp_point)

        self.global_path_pub.publish(global_node_path_msg)

    def init_node(self, x, y):
        # TODO: (2) 시작 Node 와 종료 Node 정의
        # 시작 Node 는 Rviz 기능을 이용해 지정한 위치에서 가장 가까이 있는 Node 로 한다.
        '''
        # Rviz 의 2D Pose Estimate 기능을 이용해 시작 Node를 지정합니다.
        # Rviz 창에서 2D Pose Estimate 기능 클릭 후 마우스 좌 클릭을 통해 원하는 위치를 지정할 수 있습니다.
        # 출발 위치를 2D Pose Estimate 지정 하면 Rviz 에서
        # PoseWithCovarianceStamped 형식의 ROS 메세지를 Publish 합니다.
        # 해당 형식의 메세지를 Subscribe 해서  2D Pose Estimate 로 지정한 위치와 가장 가까운 노드를 탐색하는 합니다.
        # 가장 가까운 Node 가 탐색 된다면 이를 "self.start_node" 변수에 해당 Node Idx 를 지정합니다.

        self.start_node = node_idx

        '''
        min_dis = float('inf')
        node_index = -1
        for idx, node in self.nodes.items():
            temp_dis = ((x - node.point[0]) * (x - node.point[0]) + (y - node.point[1]) * (
                    y - node.point[1])) ** 1 / 2
            if min_dis > temp_dis:
                min_dis = temp_dis
                node_index = idx
        self.is_init_pose = True
        return node_index

    def init_callback(self, msg):
        # TODO: (2) 시작 Node 와 종료 Node 정의
        # 시작 Node 는 Rviz 기능을 이용해 지정한 위치에서 가장 가까이 있는 Node 로 한다.
        '''
        # Rviz 의 2D Pose Estimate 기능을 이용해 시작 Node를 지정합니다.
        # Rviz 창에서 2D Pose Estimate 기능 클릭 후 마우스 좌 클릭을 통해 원하는 위치를 지정할 수 있습니다.
        # 출발 위치를 2D Pose Estimate 지정 하면 Rviz 에서
        # PoseWithCovarianceStamped 형식의 ROS 메세지를 Publish 합니다.
        # 해당 형식의 메세지를 Subscribe 해서  2D Pose Estimate 로 지정한 위치와 가장 가까운 노드를 탐색하는 합니다.
        # 가장 가까운 Node 가 탐색 된다면 이를 "self.start_node" 변수에 해당 Node Idx 를 지정합니다.

        self.start_node = node_idx

        '''

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        min_dis = float('inf')
        node_index = -1
        for idx, node in self.nodes.items():
            temp_dis = ((x - node.point[0]) * (x - node.point[0]) + (y - node.point[1]) * (
                    y - node.point[1])) ** 1 / 2
            if min_dis > temp_dis:
                min_dis = temp_dis
                node_index = idx
        self.start_node = node_index
        self.is_init_pose = True

    def goal_callback(self, msg):
        # TODO: (2) 시작 Node 와 종료 Node 정의
        # 종료 Node 는 Rviz 기능을 이용해 지정한 위치에서 가장 가까이 있는 Node 로 한다.
        '''
        # Rviz 의 2D Nav Goal 기능을 이용해 도착 Node를 지정합니다.
        # Rviz 창에서 2D Nav Goal 기능 클릭 후 마우스 좌 클릭을 통해 원하는 위치를 지정할 수 있습니다.
        # 도착 위치를 2D Nav Goal 지정 하면 Rviz 에서
        # PoseStamped 형식의 ROS 메세지를 Publish 합니다.
        # 해당 형식의 메세지를 Subscribe 해서  2D Nav Goal 로 지정한 위치와 가장 가까운 노드를 탐색하는 합니다.
        # 가장 가까운 Node 가 탐색 된다면 이를 "self.start_node" 변수에 해당 Node Idx 를 지정합니다.

        self.end_node = node_idx

        '''
        x = msg.pose.position.x
        y = msg.pose.position.y
        min_dis = float('inf')
        node_index = -1
        for idx, node in self.nodes.items():
            temp_dis = ((x - node.point[0]) * (x - node.point[0]) + (y - node.point[1]) * (
                    y - node.point[1])) ** 1 / 2
            if min_dis > temp_dis:
                min_dis = temp_dis
                node_index = idx
        self.end_node = node_index
        self.is_goal_pose = True

    def printPath(self, calc_dic, k, visited):
        # visited 상태에서, 남은 점들을 최적으로 돌 때, 다음으로 방문하는 점을 찾는다.
        self.path.append(k)

        if visited == (1 << self.n) - 1:
            return

        nextvalue = [float('inf'), 0]

        for i in range(self.n):
            if visited & (1 << i):
                continue
            if (k, i) not in calc_dic:
                continue
            # dp[i][visited] 현재 visited에서의 최적이 들어있고, 우리가 찾는건 다음의 최적 점이다.
            # 현재의 visited에 하나씩 비트 붙여가면서 값을 구했을 때, 그 값들 중 최솟값이 다음 최적 점이다.
            if (calc_dic[(k, i)]['path_weight'] + self.cache[i][visited | (1 << i)]) < nextvalue[0]:
                nextvalue[0] = calc_dic[(k, i)]['path_weight'] + self.cache[i][visited | (1 << i)]
                nextvalue[1] = i

        # for loop 종료 시, nextvalue[0]에는 남은 점들을 최적으로 돌았을 때의 최소 거리가
        # nextvalue[1]에는 다음 점이 들어 있다

        self.printPath(calc_dic, nextvalue[1], visited | (1 << nextvalue[1]))  # 반복.

    # 0번이 유치원
    # 마지막에 유치원으로 돌아올 때는 dijkstra로 구하는 최소 경로가 아닌 구해놓은 경로 사용
    # 외판원순회 알고리즘
    def find_path(self, calc_dic, last, visited):  # 0번이 유치원
        # 모든 도시가 켜져있다 -> 다 방문
        if visited == self.VISITED_ALL:
            # 마지막 방문 도시 출발 - 0번째 (출발 도시) 사이에 경로가 존재하면
            # 경로 값을 반환.
            # 경로가 존재하지 않는다면 무한값을 반환해서 답이 안되게 한다.
            if (last, 0) in calc_dic:
                return calc_dic[(last, 0)]['path_weight']
            else:
                return float('inf')  # 마지막 도착 도시에서 출발 도시인 0으로 가야됨.(문제 조건) # 마지막 도착 도시에서 출발 도시인 0으로 가야됨.(문제 조건)

        # cache 값이 None이 아니라는 것은 last와 visited의 계산이 이미 수행됬고,
        # 지금은 중복호출 되었다는 뜻임 -> 다시 계산하지 않고 값만 바로 반환하도록
        # 중복계산을 없애 효율성 높임 --> DP 사용하는 이유
        # 마지막으로 방문한 곳에서 방문하지 않은 아이들을 다 방문하고 도착지에 도착하는 값이 이미 존재한다
        if self.cache[last][visited] != 0:
            return self.cache[last][visited]

        tmp = float('inf')
        for city in range(self.n):
            # 방문하지 않은 도시이다 && cities 마지막방문에서 도시로의 길이 존재하다
            if visited & (1 << city) == 0 and (last, city) in calc_dic:
                # find_path(city, visited | (1<<city))
                # 마지막 방문이 city 이고 visited는 방문했는지 확인하는 비트인데 -> | 둘 중 하나라도 1이면 1을 반환(방문을 의미) 마지막 도시 방문을 표기
                # 여기서 나온 값과 현재 마지막 방문 도시에서 고른 도시까지의 값을 더한다
                # 못간다면 계속 INF 일 것이다
                tmp = min(tmp,
                          self.find_path(calc_dic, city, visited | (1 << city)) + calc_dic[(last, city)]['path_weight'])
        # 마지막 방문에서 여러 경로를 통해 마지막에 도착한 값이 tmp이다.
        self.cache[last][visited] = tmp
        return tmp

    def calc_A_star_path_node(self):

        # TODO: (10) dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'

        calc_dic = {}
        # print(self.way_point_node)
        for i in range(len(self.way_point_node)):
            for j in range(len(self.way_point_node)):
                if i == j:
                    continue
                result, path = self.global_planner.A_star(self.nodes[self.way_point_node[i]],
                                                          self.nodes[self.way_point_node[j]], self.way_point_node[i],
                                                          self.way_point_node[j])

                if result:
                    calc_dic[(i, j)] = path

        '''
        calc_dic  {'node_path': total_path, 'link_path': link_path, 'point_path': point_path,
                                      'path_weight': total_weight}

        '''

        # dijkstra 수행
        # stop_list : 정류장(node) 순서
        self.n = len(self.way_point_node)
        self.VISITED_ALL = (1 << self.n) - 1
        self.cache = [[0] * (1 << self.n) for _ in range(self.n)]
        final_result = self.find_path(calc_dic, 0, 1 << 0)

        self.printPath(calc_dic, 0, 1 << 0)

        # global path 생성 (rviz에 찍을 거)
        # 정류장 순서대로 시작정류장과 도착정류장 사이의 point값
        # [[0번에서1번 포인트들][1번에서 2번 포인트들][...][...]]
        node_path = []
        for i in range(len(self.path)):
            self.node_order.append((self.nodes[self.way_point_node[self.path[i]]].point[0], self.nodes[self.way_point_node[self.path[i]]].point[1])) # node_idx로부터 x,y 구하기

            tmp_path = Path()
            tmp_path.header.frame_id = '/map'  # 이거 안 쓰면 rviz 화면에 안 찍힘
            if i == len(self.path) - 1:
                v = self.path[0]
                try:
                    for idx in calc_dic[(self.path[i], v)]['point_path']:
                        tmp_point = PoseStamped()
                        tmp_point.pose.position.x = float(idx[0])
                        tmp_point.pose.position.y = float(idx[1])
                        tmp_point.pose.orientation.w = 1
                        out_path.poses.append(tmp_point)
                        tmp_path.poses.append(tmp_point)
                except:
                    break

            else:
                try:
                    for idx in calc_dic[(self.path[i], self.path[i + 1])]['point_path']:
                        tmp_point = PoseStamped()
                        tmp_point.pose.position.x = float(idx[0])
                        tmp_point.pose.position.y = float(idx[1])
                        tmp_point.pose.orientation.w = 1
                        out_path.poses.append(tmp_point)
                        tmp_path.poses.append(tmp_point)
                except:
                    break

            node_path.append(tmp_path)

        return out_path, node_path


class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    # heuristic으로 추정
    # 현재(current)와 목적지 (goal)간의 직선 거리로 추정
    def heuristic_cost(self, start, goal):
        return sqrt(pow(start.point[0] - goal.point[0], 2) + pow(start.point[1] - goal.point[1], 2))

    # 경로 생성
    def reconstruct_path(self, cameFrom, current):
        total_path = [current]
        cur = current
        while cur in cameFrom.keys():
            cur = cameFrom[cur]
            total_path.append(cur)
        total_path.reverse()
        return total_path

    def A_star(self, start, goal, start_node, end_node):

        # print("ASTAR START => " + str(start_node) + " " + str(end_node))
        closeSet = []  # 닫힌 목록
        openSet = [start_node]  # 열린 목록, 초기에 시작 노드만 있음

        cameFrom = {}  # 가장 효율적인 경로
        gScore = {}  # 시작 노드로부터 비용
        link_path = []  # 경로들의 간선 모음집

        # 각 노드별로 시작 노드로부터 거리
        for i in self.nodes.keys():
            gScore[i] = float('inf')

        # 시작 노드이므로
        gScore[start_node] = 0

        # 시작 노드로부터 비용 + 목적 노드까지 비용
        fScore = {}
        for i in self.nodes.keys():
            fScore[i] = float('inf')

        # 첫번째 노드 : 시작 노드로부터 0, 목적 노드까지 heuristic한 *추정* 비용
        # 첫번째 노드의 전체 비용은 추정값
        # heuristic 추정 => start, goal 직선 거리 계산
        start_cost = self.heuristic_cost(start, goal)
        fScore[start_node] = start_cost

        # 열린 목록이 빌 때까지 반복
        while True:
            if len(openSet) == 0:
                print("len(openSet) == 0")
                return False, {'node_path': [], 'link_path': [], 'point_path': [],
                               'path_weight': 0}
                break
            # 열린 목록에서 가장 작은 f 값 가지는 노드
            current = openSet[0]  # current => node_idx
            for node in openSet:  # node => node_idx
                if node == current:
                    continue
                if fScore[node] < fScore[current]:
                    current = node

            # print("CURRENT => " + str(current))
            if current == end_node:  # 목적 노드라면 경로 생성
                total_path = self.reconstruct_path(cameFrom, current)
                total_weight = 0
                # link_path 얻기
                for i in range(len(total_path) - 1):
                    from_node_idx = total_path[i]
                    to_node_idx = total_path[i + 1]

                    from_node = self.nodes[from_node_idx]
                    to_node = self.nodes[to_node_idx]

                    shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node, to_node)
                    total_weight += min_cost
                    link_path.append(shortest_link.idx)
                if len(link_path) == 0:
                    return False, {'node_path': total_path, 'link_path': link_path, 'point_path': [],
                                   'path_weight': total_weight}

                point_path = []
                for link_id in link_path:
                    link = self.links[link_id]
                    for point in link.points:
                        point_path.append([point[0], point[1], 0])
                return True, {'node_path': total_path, 'link_path': link_path, 'point_path': point_path,
                              'path_weight': total_weight}

            # 목적 노드 아니라면 열린 목록에서 삭제하고 닫힌 목록에 추가
            openSet.remove(current)
            closeSet.append(current)

            # current(가장 작은 f 값 가지는 노드)의 인접 노드 보기
            # 인접한 애들 다 받기
            # node_neighbor가 인접한 애들
            for node_neighbor in self.nodes[current].get_to_nodes():
                # 인접한 애들과 current 간의 거리 구한다.
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(self.nodes[current], node_neighbor)
                if (node_neighbor == None): continue
                neighbor = node_neighbor.idx

                # 닫힌 목록에 있다면 pass
                if neighbor in closeSet:
                    continue
                # 열린 목록에 없다면 추가
                if neighbor not in openSet:
                    openSet.append(neighbor)

                # (current 노드까지 gScore) + (current와 neighbor까지의 거리)
                # 시작 노드부터 neighbor까지가
                # 시작 노드부터 current 거쳐서 neighbor까지 가는 것보다 짧다면 무시
                tentative_gScore = gScore[current] + min_cost
                if tentative_gScore >= gScore[neighbor]:
                    continue

                # current 거치는 게 더 짧다면 기록
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore  # 갱신
                # heuristic
                # 인접한 애들로 부터 목적지(goal)까지 직선거리
                min_cost = self.heuristic_cost(node_neighbor, goal)
                # 갱신
                fScore[neighbor] = gScore[neighbor] + min_cost

    def get_weight_matrix(self):
        # TODO: (3) weight 값 계산
        '''
        가중치 계산
        # weight 값 계산은 각 Node 에서 인접 한 다른 Node 까지의 비용을 계산합니다.
        # 계산된 weight 값 은 각 노드간 이동시 발생하는 비용(거리)을 가지고 있기 때문에
        # Dijkstra 탐색에서 중요하게 사용 됩니다.
        # weight 값은 딕셔너리 형태로 사용 합니다.
        # 이중 중첩된 딕셔너리 형태로 사용하며
        # Key 값으로 Node의 Idx Value 값으로 다른 노드 까지의 비용을 가지도록 합니다.
        # 아래 코드 중 self.find_shortest_link_leading_to_node 를 완성하여
        # Dijkstra 알고리즘 계산을 위한 Node와 Node 사이의 최단 거리를 계산합니다.

        '''
        # 초기 설정
        weight = dict()
        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 다른 노드로 진행하는 모든 weight
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            # 전체 weight matrix에 추가
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 현재 노드로는 cost = 0
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node, to_node)
                if (to_node == None): continue
                weight[from_node_id][to_node.idx] = min_cost

        return weight

    def find_shortest_link_leading_to_node(self, from_node, to_node):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        # TODO: (3) weight 값 계산
        '''
        # 최단거리 Link 인 shortest_link 변수와
        # shortest_link 의 min_cost 를 계산 합니다.

        '''
        shortest_link, min_cost = from_node.find_shortest_link_leading_to_node(to_node)
        return shortest_link, min_cost

    def find_nearest_node_idx(self, distance, s):
        idx_list = self.nodes.keys()
        min_value = float('inf')
        min_idx = idx_list[-1]

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False:
                min_value = distance[idx]
                min_idx = idx
        return min_idx


if __name__ == '__main__':
    dijkstra_path_pub = dijkstra_path_pub()