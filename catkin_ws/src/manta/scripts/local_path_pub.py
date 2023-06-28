#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String

class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        #TODO: (1) Global Path 와 Odometry 데이터 subscriber 생성 
        '''
        # Global Path 와 Odometry 데이터 subscriber 를 생성한다.
        # 콜백 함수의 이름은 self.global_path_callback, self.odom_callback 로 한다.
        rospy.Subscriber( odometry 메세지 콜백 완성하기 )
        rospy.Subscriber( global path 메세지 콜백 완성하기 )

        '''
        #rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Odometry가 현재 나의 위치에 대한 정보
        # 전체 받기
        rospy.Subscriber('/global_path', Path, self.real_global_path_callback)
        # 지역 받기
        # rospy.Subscriber('/real_global_path', Path, self.real_global_path_callback)

        # current_waypoint 0으로 갱신하라
        rospy.Subscriber('/local_current_waypoint_update', String, self.local_waypoint_callback)

        # 현재 내가 쫓아가는 포인트 번호 publish
        self.local_point_pub = rospy.Publisher("/local_point_pub", String, queue_size=10)

        #TODO: (2) Local Path publisher 선언
        '''
        # local Path 데이터를 Publish 하는 변수를 선언한다.
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)

        '''
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)

        # 초기화
        self.is_odom = False
        self.is_path = False

        #TODO: (3) Local Path 의 Size 결정
        '''
        # Local Path 의 크기를 지정한다.
        # 차량이 주행 시 Local Path 의 크기 만큼의 정보를 가지고 주행하게 된다
        # 너무 작지도 크기지도 않은 값을 사용한다 (50 ~ 200)
        self.local_path_size = 

        '''
        self.local_path_size = 100

        rate = rospy.Rate(20) # 20hz
        self.current_waypoint = 0
        while not rospy.is_shutdown():
            if self.is_odom == True and self.is_path == True:
                local_path_msg = Path()
                local_path_msg.header.frame_id='/map'
                

                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색
                '''
                # global Path 에서 차량의 현재 위치를 찾습니다.
                # 현제 위치는 WayPoint 로 기록하며 현재 차량이 Path 에서 몇번 째 위치에 있는지 나타내는 값이 됩니다.
                # 차량의 현재 위치는 Local Path 를 만드는 시작 위치가 됩니다.
                # 차량의 현재 위치를 탐색하는 반복문은 작성해 current_waypoint 찾습니다.
                min_dis = float('inf')
                current_waypoint = -1
                for  in  :

                '''
                # print(self.global_path_msg.poses)
                # min_dis = float('inf')
                # current_waypoint = -1
                # for idx, path in enumerate(self.global_path_msg.poses):
                #     # print(path)
                #     temp_dis = ((x-path.pose.position.x)*(x-path.pose.position.x)+(y-path.pose.position.y)*(y-path.pose.position.y))**1/2
                #     if min_dis > temp_dis:
                #         min_dis = temp_dis
                #         current_waypoint = idx



                if self.current_waypoint_index == 1:
                    find_length = len(self.real_global_path_msg.poses)
                else:
                    find_length = 1000

                tmp_waypoint = self.current_waypoint

                min_dis = float('inf')
                for i in range(find_length):
                    if tmp_waypoint == len(self.real_global_path_msg.poses)-1:
                        tmp_waypoint = 0
                        continue
                    try:
                        tmp_dis = sqrt(pow(self.real_global_path_msg.poses[tmp_waypoint].pose.position.x - x, 2) + pow(
                            self.real_global_path_msg.poses[tmp_waypoint].pose.position.y - y, 2))
                    except IndexError:
                        print(len(self.real_global_path_msg.poses))
                        print(tmp_waypoint)

                    if min_dis > tmp_dis:
                        self.current_waypoint = tmp_waypoint
                        min_dis = tmp_dis
                    tmp_waypoint += 1
                self.local_point_pub.publish(str(self.current_waypoint))  # 포인트 idx 보내기

                #TODO: (6) 가장 가까운 포인트(current Waypoint) 위치부터 Local Path 생성 및 예외 처리
               
                if self.current_waypoint != -1:
                    if self.current_waypoint + self.local_path_size < len(self.real_global_path_msg.poses):
                        for i in range(self.current_waypoint, self.current_waypoint + self.local_path_size):
                            local_path_msg.poses.append(self.real_global_path_msg.poses[i])
                    else:
                        for i in range(self.current_waypoint, len(self.real_global_path_msg.poses)):
                            local_path_msg.poses.append(self.real_global_path_msg.poses[i])

                # print(x,y)
                #TODO: (7) Local Path 메세지 Publish
                '''
                # Local Path 메세지 를 전송하는 publisher 를 만든다.
                self.local_path_pub.
                
                '''
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장

        '''
        # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
        # Odometry 메세지 에 담긴 물체의 위치 데이터를 아래 변수에 넣어준다.
        self.x = 물체의 x 좌표 
        self.y = 물체의 y 좌표

        '''
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg
        # print("global_path_msg")
        # header / poses
        # poses 안에 header, pose.position, pose.orientation 이 들어있다.
        # 아무래도 poses의 크기가 node의 갯수로 보인다.
        # print(self.global_path_msg)
        # print("poses")
        # print(self.global_path_msg.poses)
        # print(len(self.global_path_msg.poses))
        # 4231 개의 노드가 들어있다

    def real_global_path_callback(self,msg):
        self.is_path = True
        self.real_global_path_msg = msg

    def local_waypoint_callback(self, msg):
        # current_waypoint 0으로 갱신
        # print(msg)
        # self.current_waypoint_node_ak = int(msg.data)
        self.current_waypoint_index = int(msg.data)
        # print(self.current_waypoint)

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
