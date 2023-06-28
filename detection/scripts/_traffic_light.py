#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image_lane_roi 는 카메라 센서를 통하여 받아온 이미지에 관심있는 부분만(차선) 만 남기고
# 나머지 부분은 마스킹 하는 이미리 처리입니다. 관심 영역을 지정하고, 마스크를 생성, 마스크를 이미지에 합치는 과정을
# 합니다. 

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed_T", CompressedImage, self.callback)
        
        # image_size
        x = 640
        y = 480

        #TODO: (1) 관심있는 영역만 지정.
        '''
        4개의 포인트를 지정
        이미지의 좌표를 직접 지정해도 되고,
        이미지의 비율로 정의해도 됩니다.
        np.array 사용
        self.crop_pts =

        '''
        self.crop_pts = np.array(
            [[
            [0,0],
            [640,0],
            [640,450],
            [0,450],
            ]]
        )

        # lower_yellow = np.array([15,  100, 100])
        # upper_yellow = np.array([30, 255, 255])

        # lower_green = np.array([15,  100, 100])
        # upper_green = np.array([30, 255, 255])

    def callback(self, msg):
        # uint8 : unsined integer 0~255 로 만들기 위함입니다.
        lower_red1 = np.array([0, 0, 0])
        upper_red1 = np.array([20, 255, 255])
        lower_red2 = np.array([160, 0, 0])
        upper_red2 = np.array([180, 255, 255])

        lower_yellow = np.array([20, 0, 0])
        upper_yellow = np.array([40, 255, 255])

        lower_green = np.array([40, 0, 0])
        upper_green = np.array([70, 255, 255])
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)

        self.mask = self.mask_roi(img_hsv)

        img_red1 = cv2.inRange(self.mask, lower_red1, upper_red1)
        img_red2 = cv2.inRange(self.mask, lower_red2, upper_red2)

        img_red = cv2.bitwise_or(img_red1, img_red2)

        img_yellow = cv2.inRange(self.mask, lower_yellow, upper_yellow)
        
        img_green = cv2.inRange(self.mask, lower_green, upper_green)

        cv2.imshow("Image window1", img_bgr)
        cv2.imshow("Image window2", img_red)
        cv2.imshow("Image window3", img_yellow)
        cv2.imshow("Image window4", img_green)
        cv2.waitKey(1)

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        # img.shape == [H,W,C]의 3차원이고 C RGB를 갖는 3채널입니다. RGB 이미지 입니다.
        if len(img.shape)==3:

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        # grayscale image일 경우. (시뮬레이터에서 주는 이미지는 항상 3차원이기 때문에 예외를 위해서 만들어 놓은 부분 입니다.)
        else:

            mask = np.zeros((h, w), dtype=np.uint8)

            mask_value = (255)
        
        # TODO (1) 에서 마스킹 영역을 만들었고, 관심 있는 부분만을 이미지 원본으로 하고 나머지는 255(검은색)로 반환 해 주는
        #내용이 들어가야 합니다.
        #마스킹 영역을 만들기 위해서 다양한 방법을 사용할 수 있습니다만, 코드에서 이미 까만 이미지를 생성했습니다.
        #이를 이용하는 방법을 찾아야 합니다.
        
        #TODO: (2) 
        '''
        먼저 원하는 만큼의 좌표 점들을 선으로 긋고, 시작점과 끝점을 자동으로 연결하여 다각형을 그리는 함수를 opencv 함수를
        찾습니다.
        cv2.
        '''

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        #TODO : (3)
        '''
        # 다음으로 RGB 이미지를 마스킹 하는 opencv 함수를 이용합니다. 비트연산을 하는 함수이며, 0,1을 이용하는 연산으로
        두 이미지의 동일한 위치에 대한 연산을 진행합니다.
        mask = cv2.
        '''
        
        mask = cv2.bitwise_and(mask, img)
        return mask


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 