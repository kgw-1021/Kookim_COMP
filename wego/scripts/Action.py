#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from morai_msgs.msg import GetTrafficLightStatus
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
from math import *
import time
from reference import*

class Action:
    def __init__(self):
        rospy.init_node('Action')
        self.bridge = CvBridge()        
        self.control = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.state = 0
        rospy.Subscriber("/action_flag", Float64, self.state_CB) 
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_CB)
        rospy.Subscriber("/GetTrafficLightStatus",GetTrafficLightStatus,self.traffic_CB)
        rospy.Subscriber("/lidar2D",LaserScan,self.lidar_CB)
        self.image_msg = CompressedImage()
        self.traffic_msg=GetTrafficLightStatus()
        self.nothing_flag = False
        self.cmd_msg = Twist()
        self.nwindows = 10
        self.window_height = 480//self.nwindows
        # self.img_pub = rospy.Publisher("/perspective/compressed",CompressedImage,queue_size=10)
        self.flag = 0
        self.good_left_idx = []
        self.good_right_idx = []
        self.static_count = 0
        self.cross_flag=0
        self.cross_flag2=0
        self.lidar_flag=0
        self.flag=0#정지선 검출용
        self.flagship=0 #정지선을 한번 넘어갈때마다 1개씩 축적, 이걸로 언제 좌회전할지 정할수 있음. # 정지선
        self.flag2=0
        self.flagship2=0 # 로터리 정지선
        self.signal=0
        self.move=0 # 신호등
        self.dst = 2
        self.flagship2_start_time = None
        self.flagship1_start_time = None
        self.rate = rospy.Rate(30)

    ###### Call Back Func Def ######        

    def state_CB(self, data):
        self.state = data.data
    
    def img_CB(self, data):
        self.img = self.bridge.compressed_imgmsg_to_cv2(data)

    def lidar_CB(self,msg):
        if self.state == 3:
            self.scan_msg = msg
            degree_min = self.scan_msg.angle_min * 180/pi
            degree_max = self.scan_msg.angle_max * 180/pi
            degree_anle_increment = self.scan_msg.angle_increment * 180/pi

            degrees = [degree_min + degree_anle_increment * index for index, value in enumerate(self.scan_msg.ranges)] # 각도 값[도]
            obstacle_degrees = []
            for index , value in enumerate(self.scan_msg.ranges):
                if not(-135 < degrees[index] < 135) and 0 < value < self.dst: # 각도 값, 거리 값 함께 고려
                    # print(f"{index} : {degrees[index]}") # 이쪽 인덱스, 각도에 장애물
                    obstacle_degrees.append(degrees[index])
                else:
                    pass
            
            # print(len(obstacle_degrees))
            # print(obstacle_degrees)
            if len(obstacle_degrees) > 0:
                self.lidar_flag = 1
            else:
                self.lidar_flag = 0

    def traffic_CB(self,msg):
        if self.state == 4:
            self.traffic_msg=msg


    ###### LKAS func Def ######
    
    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        yellow_lower = np.array([15, 80, 0])
        yellow_upper = np.array([45, 255, 255])

        # Define range of blend color in HSV
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 64, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # Threshold the HSV image to get only white colors
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # Threshold the HSV image to get blend colors
        blend_mask = cv2.bitwise_or(yellow_mask, white_mask)
        blend_color = cv2.bitwise_and(img, img, mask=blend_mask)
        return blend_color

    def img_warp(self, img):
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        # print(f'self.img_x:{self.img_x}, self.img_y:{self.img_y}')

        img_size = [640, 480]
        # ROI
        src_side_offset = [0, 240]
        src_center_offset = [220, 315]
        src = np.float32(
            [
                [0, 479],
                [src_center_offset[0]-10, src_center_offset[1]+20],
                [640 - src_center_offset[0]+10, src_center_offset[1]+20],
                [639, 479],
            ]
        )
        # 아래 2 개 점 기준으로 dst 영역을 설정합니다.
        dst_offset = [round(self.img_x * 0.125), 0]
        # offset x 값이 작아질 수록 dst box width 증가합니다.
        dst = np.float32(
            [
                [dst_offset[0]+10, self.img_y],
                [dst_offset[0]+60, 0],
                [self.img_x - dst_offset[0]-60, 0],
                [self.img_x - dst_offset[0]-10, self.img_y],
            ]
        )
        # find perspective matrix
        matrix = cv2.getPerspectiveTransform(src, dst)
        # matrix_inv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, matrix, (self.img_x, self.img_y))
        return warp_img
    
    def img_binary(self, blend_line):
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin > 50] = 1
        return binary_line

    def detect_nothing(self):
        self.nothing_left_x_base = round(self.img_x * 0.140625)
        self.nothing_right_x_base = self.img_x - round(self.img_x * 0.140625)

        self.nothing_pixel_left_x = np.zeros(self.nwindows) + round(self.img_x * 0.140625)
        self.nothing_pixel_right_x = np.zeros(self.nwindows) + self.img_x - round(self.img_x * 0.140625)
        self.nothing_pixel_y = [round(self.window_height / 2) * index for index in range(0, self.nwindows)]

    def window_search(self, binary_line):
        # histogram을 생성합니다.
        # y축 기준 절반 아래 부분만을 사용하여 x축 기준 픽셀의 분포를 구합니다.
        bottom_half_y = binary_line.shape[0] / 2
        histogram = np.sum(binary_line[int(bottom_half_y) :, :], axis=0)
        # 히스토그램을 절반으로 나누어 좌우 히스토그램의 최대값의 인덱스를 반환합니다.
        midpoint = np.int32(histogram.shape[0] / 2)
        # left_x_base = 80
        # right_x_base = 640 - 80
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint
        # show histogram
        # plt.hist(histogram)
        # plt.show()
        if left_x_base == 0:
            left_x_current = self.nothing_left_x_base
        else:
            left_x_current = left_x_base
        if right_x_base == midpoint:
            right_x_current = self.nothing_right_x_base
        else:
            right_x_current = right_x_base

        out_img = np.dstack((binary_line, binary_line, binary_line)) * 255

        ## window parameter
        # 적절한 윈도우의 개수를 지정합니다.
        nwindows = self.nwindows
        # 개수가 너무 적으면 정확하게 차선을 찾기 힘듭니다.
        # 개수가 너무 많으면 연산량이 증가하여 시간이 오래 걸립니다.
        window_height = self.window_height
        # 윈도우의 너비를 지정합니다. 윈도우가 옆 차선까지 넘어가지 않게 사이즈를 적절히 지정합니다.
        margin = 25
        # 탐색할 최소 픽셀의 개수를 지정합니다.
        min_pix = round((margin * window_height) * 0.0031)

        lane_pixel = binary_line.nonzero()
        lane_pixel_y = np.array(lane_pixel[0])
        lane_pixel_x = np.array(lane_pixel[1])

        # pixel index를 담을 list를 만들어 줍니다.
        left_lane_idx = []
        right_lane_idx = []

        # Step through the windows one by one
        for window in range(nwindows):
            # window boundary를 지정합니다. (세로)
            win_y_low = binary_line.shape[0] - (window + 1) * window_height
            win_y_high = binary_line.shape[0] - window * window_height
            # print("check param : \n",window,win_y_low,win_y_high)

            # position 기준 window size
            win_x_left_low = left_x_current - margin
            win_x_left_high = left_x_current + margin
            win_x_right_low = right_x_current - margin
            win_x_right_high = right_x_current + margin

            # window 시각화입니다.
            if left_x_current != 0:
                cv2.rectangle(
                    out_img,
                    (win_x_left_low, win_y_low),
                    (win_x_left_high, win_y_high),
                    (0, 255, 0),
                    2,
                )
            if right_x_current != midpoint:
                cv2.rectangle(
                    out_img,
                    (win_x_right_low, win_y_low),
                    (win_x_right_high, win_y_high),
                    (0, 0, 255),
                    2,
                )


            # 왼쪽 오른쪽 각 차선 픽셀이 window안에 있는 경우 index를 저장합니다.
            if self.flag == 0:
                self.good_left_idx = (
                    (lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) & (lane_pixel_x >= win_x_left_low) & (lane_pixel_x < win_x_left_high)
                ).nonzero()[0]
                self.good_right_idx = (
                    (lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) & (lane_pixel_x >= win_x_right_low) & (lane_pixel_x < win_x_right_high)
                ).nonzero()[0]
            elif self.flag == 1:
                self.good_left_idx = (
                    (lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) & (lane_pixel_x >= win_x_left_low) & (lane_pixel_x < win_x_left_high)
                ).nonzero()[0]
            elif self.flag == 2:
                self.good_right_idx = (
                    (lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) & (lane_pixel_x >= win_x_right_low) & (lane_pixel_x < win_x_right_high)
                ).nonzero()[0]

            # Append these indices to the lists
            left_lane_idx.append(self.good_left_idx)
            right_lane_idx.append(self.good_right_idx)

            # window내 설정한 pixel개수 이상이 탐지되면, 픽셀들의 x 좌표 평균으로 업데이트 합니다.
            if len(self.good_left_idx) > min_pix:
                left_x_current = np.int32(np.mean(lane_pixel_x[self.good_left_idx]))
            if len(self.good_right_idx) > min_pix:
                right_x_current = np.int32(np.mean(lane_pixel_x[self.good_right_idx]))

        # np.concatenate(array) => axis 0으로 차원 감소 시킵니다.(window개수로 감소)
        left_lane_idx = np.concatenate(left_lane_idx)
        right_lane_idx = np.concatenate(right_lane_idx)

        # window 별 좌우 도로 픽셀 좌표입니다.
        left_x = lane_pixel_x[left_lane_idx]
        left_y = lane_pixel_y[left_lane_idx]
        right_x = lane_pixel_x[right_lane_idx]
        right_y = lane_pixel_y[right_lane_idx]

        # 좌우 차선 별 2차 함수 계수를 추정합니다.
        if len(left_x) == 0 and len(right_x) == 0:
            left_x = self.nothing_pixel_left_x
            left_y = self.nothing_pixel_y
            right_x = self.nothing_pixel_right_x
            right_y = self.nothing_pixel_y
        else:
            if len(left_x) == 0:
                left_x = right_x - (round(self.img_x / 2) + 150)
                left_y = right_y
            elif len(right_x) == 0:
                right_x = left_x + (round(self.img_x / 2) + 150)
                right_y = left_y

        left_fit = np.polyfit(left_y, left_x, 2)
        right_fit = np.polyfit(right_y, right_x, 2)
        # 좌우 차선 별 추정할 y좌표입니다.
        plot_y = np.linspace(0, binary_line.shape[0] - 1, 100)
        # 좌우 차선 별 2차 곡선을 추정합니다.
        left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
        right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]
        center_fit_x = (right_fit_x + left_fit_x) / 2

        # # window안의 lane을 black 처리합니다.
        # out_img[lane_pixel_y[left_lane_idx], lane_pixel_x[left_lane_idx]] = (0, 0, 0)
        # out_img[lane_pixel_y[right_lane_idx], lane_pixel_x[right_lane_idx]] = (0, 0, 0)

        # 양쪽 차선 및 중심 선 pixel 좌표(x,y)로 변환합니다.
        center = np.asarray(tuple(zip(center_fit_x, plot_y)), np.int32)
        # right = np.asarray(tuple(zip(right_fit_x, plot_y)), np.int32)
        # left = np.asarray(tuple(zip(left_fit_x, plot_y)), np.int32)

        # cv2.polylines(out_img, [left], False, (0, 0, 255), thickness=5)
        # cv2.polylines(out_img, [right], False, (0, 255, 0), thickness=5)
        sliding_window_img = out_img
        return sliding_window_img, center
            
    def LKAS(self):
        if hasattr(self, 'img'):
            y,x,_ = self.img.shape

            warp_img = self.img_warp(self.img)
            blend_color = self.detect_color(warp_img)
            binary_line = self.img_binary(blend_color)
                
            
            if self.nothing_flag == False:
                self.detect_nothing()
                self.nothing_flag = True
            (
                sliding_window_img,
                center,
            ) = self.window_search(binary_line)
            

            standard_line = x//2
            center_index = np.mean(center[:,0])
            degree_per_pixel = 1/x
            self.cmd_msg.angular.z = (center_index - standard_line)*degree_per_pixel # -0.5 ~ 0.5
            weight = 3.1
            self.cmd_msg.angular.z = 0.5 + weight*self.cmd_msg.angular.z # 0 ~ 1
            # print("---------------")
            # print(center_index)
            # print(self.cmd_msg.angular.z)

            # tmp = self.bridge.cv2_to_compressed_imgmsg(sliding_window_img)
            # self.img_pub.publish(tmp)
            self.cmd_msg.angular.z = (self.cmd_msg.angular.z - 0.5) / -1.177
            self.cmd_msg.linear.x = 0.8
            self.control.publish(self.cmd_msg)
        else:
            rospy.sleep(0.5)


    ###### Roundabout & Intersection func def ######
        
    def color(self, img):  #하얀선과 노란선을 추출하여 띄우기
        # -> HSV 
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        yellow_lower = np.array([20, 100, 100]) #노란색만 검출되게 수정완료, 초록이랑 하양 겹치게 하는 부분 수정함
        yellow_upper = np.array([30, 255, 255])

        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 64, 255])

        yellow_range = cv2.inRange(hsv, yellow_lower, yellow_upper)
        
        white_range = cv2.inRange(hsv, white_lower, white_upper)
        
        combined_range= cv2.bitwise_or(yellow_range, white_range)
        self.filtered_img = cv2.bitwise_and(img, img, mask=combined_range)
        return self.filtered_img

    def warp(self, img): #이미지를 버드와이저로 변환
        self.x, self.y = self.img.shape[1], self.img.shape[0] #480,640
        # ROi
        src_center_offset = [210, 315]

        src_point1=[0, 479]
        src_point2=[src_center_offset[0], src_center_offset[1]]
        src_point3=[640 - src_center_offset[0], src_center_offset[1]]
        src_point4=[639, 479]
        src=np.float32([src_point1,src_point2,src_point3,src_point4])

        dst_offset = [round(self.x * 0.125), 0]
         # offset x 값이 작아질 수록 dst box width 증가합니다.

        dst_point1=[dst_offset[0], self.y]
        dst_point2=[dst_offset[0], 0]
        dst_point3=[self.x - dst_offset[0], 0]
        dst_point4=[self.x - dst_offset[0], self.y]
        dst=np.float32([dst_point1,dst_point2,dst_point3,dst_point4])
        
        # find perspective matrix
        matrix = cv2.getPerspectiveTransform(src, dst)
        self.warp_img = cv2.warpPerspective(img, matrix, (self.x, self.y))
        return self.warp_img

    def binary(self,filtered_img): #bin으로 변환
        grayed_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        self.bin_img = np.zeros_like(grayed_img)
        self.bin_img[grayed_img !=0] = 1
        return self.bin_img
    
    def line_dect(self,img):
        histogram=np.sum(img,axis=0)
        histogram_y=np.sum(img,axis=1)
        
        downhistogram_y=histogram_y[:self.y//4*3]
        cross_indices=np.where(downhistogram_y>400)[0]
        cross_indices2=np.where((400 > downhistogram_y) & (downhistogram_y > 250))[0]
        # try:
        #     # print(cross_indices2)
        #     # print(histogram_y)
        #     print(len(cross_indices2))
        # except:
        #     pass
        # print(cross_indices)
        # dif=cross_indices[-1]-cross_indices[0]
        try:
            cross_threshold=100
            if cross_threshold<(cross_indices[-1]-cross_indices[0]):
                self.cross_flag=True
                self.flag+=1
                cv2.rectangle(self.filtered_img,[0,cross_indices[0]],[self.x,cross_indices[-1]],[0,255,0],3)
            else:
                self.cross_flag=False
                if self.flag!=0:
                    self.flag=0
                    self.flagship+=1
        except:
           self.cross_flag=False
        try:
            cross_threshold=100
            if cross_threshold<len(cross_indices2):
                self.cross_flag2=True
                self.flag2+=1
                cv2.rectangle(self.filtered_img,[0,cross_indices2[0]],[self.x,cross_indices2[-1]],[0,0,255],3)
                
            else:
                self.cross_flag2=False
                if self.flag2!=0:
                    self.flag2=0
                    self.flagship2+=1
        except:
           self.cross_flag2=False

    def Roundabout(self):
        while not rospy.is_shutdown():
            if hasattr(self, 'img'):
                self.warp(self.img)
                self.color(self.warp_img)
                self.binary(self.filtered_img)
                self.line_dect(self.bin_img)
                self.cmd_msg.angular.z = 0.0
                self.cmd_msg.linear.x = 0.885
                if self.flagship2 == 1 and self.lidar_flag ==0 :
                    self.dst = 0.1
                    if self.flagship2_start_time is None:
                        self.flagship2_start_time = rospy.get_time()
                    elapsed_time2 = rospy.get_time() - self.flagship2_start_time
                    if elapsed_time2 <= 0.3:
                        self.cmd_msg.angular.z = 0
                        self.cmd_msg.linear.x = 0.708
                    elif 0.3 < elapsed_time2 <= 1.2:  # 1초 동안
                        self.cmd_msg.angular.z = 1.8 / -1.77
                        self.cmd_msg.linear.x = 0.885
                    elif 1.2 < elapsed_time2 <= 2.0 :
                        self.cmd_msg.angular.z = 0
                        self.cmd_msg.linear.x = 0.885
                    elif 2.0 < elapsed_time2 <= 2.9:
                        self.cmd_msg.angular.z = 1.8 / -1.77
                        self.cmd_msg.linear.x = 0.885
                    elif 2.9 < elapsed_time2:
                        break
                elif self.flagship2==1 and self.lidar_flag==1:
                    self.cmd_msg.angular.z = 0
                    self.cmd_msg.linear.x = 0
                self.control.publish(self.cmd_msg)
            else:
                rospy.sleep(0.1)

    def Intersection(self):
        while not rospy.is_shutdown():
            if hasattr(self, 'img'):
                self.warp(self.img)
                self.color(self.warp_img)
                self.binary(self.filtered_img)
                self.line_dect(self.bin_img)
                self.cmd_msg.angular.z = 0.0
                self.cmd_msg.linear.x = 0.7
                if self.traffic_msg.trafficLightIndex == "SN000005":
                    self.signal=self.traffic_msg.trafficLightStatus

                if self.flagship == 1:
                    if self.signal == 33:
                        if self.flagship1_start_time is None:
                            self.flagship1_start_time = rospy.get_time()
                        elapsed_time1 = rospy.get_time() - self.flagship1_start_time
                        if elapsed_time1 <= 1.4:
                            self.cmd_msg.angular.z = -0.5 / -1.77
                            self.cmd_msg.linear.x = 0.885
                        elif 1.4 < elapsed_time1 <= 3.5:
                            self.cmd_msg.angular.z = -1.2 / -1.77
                            self.cmd_msg.linear.x = 0.885
                        elif 3.5 < elapsed_time1:
                            break
                    else:
                        self.cmd_msg.angular.z = 0.0
                        self.cmd_msg.linear.x = 0
                self.control.publish(self.cmd_msg)
            else:
                rospy.sleep(0.1)

    ###### Turn & Change lane & etc func def ######
                        
    def straight(self):
        start_time_straight = rospy.Time.now()
        while (rospy.Time.now() - start_time_straight).to_sec() < 4:
            self.cmd_msg.angular.z = 0
            self.cmd_msg.linear.x = 1
            self.control.publish(self.cmd_msg)
            self.rate.sleep()

    def left(self):
    # Go straight for 1 second
        start_time_straight = rospy.Time.now()
        while (rospy.Time.now() - start_time_straight).to_sec() < 0.7:
            self.cmd_msg.angular.z = 0.0
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)
            self.rate.sleep()

        # Maintain left turn for 1 second
        start_time_turn = rospy.Time.now()
        while (rospy.Time.now() - start_time_turn).to_sec() < 2.8:
            self.cmd_msg.angular.z = (-0.1 - 0.5) / -1.177
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)
            self.rate.sleep()

        start_time_turn = rospy.Time.now()
        while (rospy.Time.now() - start_time_turn).to_sec() < 0.6:
            self.cmd_msg.angular.z = (-0.1 - 0.59) / -1.177
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)
            self.rate.sleep()

    def right(self):
        start_time_straight = rospy.Time.now()
        while (rospy.Time.now() - start_time_straight).to_sec() < 0.84:
            self.cmd_msg.angular.z = 0
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)
            self.rate.sleep()

        # Maintain left turn for 1 second
        start_time_turn = rospy.Time.now()
        while (rospy.Time.now() - start_time_turn).to_sec() < 2.4:
            self.cmd_msg.angular.z = (1.4 - 0.5) / -1.177
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)
            self.rate.sleep()

    def lane_change_l(self):
        start_time_straight = rospy.Time.now()
        while (rospy.Time.now() - start_time_straight).to_sec() < 1.0:
            self.cmd_msg.angular.z = -0.9 / -1.77
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)

        # Maintain left turn for 1 second
        start_time_turn = rospy.Time.now()
        while (rospy.Time.now() - start_time_turn).to_sec() < 0.4:
            self.cmd_msg.angular.z = 1.1 / -1.177
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)

    def lane_change_r(self):
        start_time_straight = rospy.Time.now()
        while (rospy.Time.now() - start_time_straight).to_sec() < 1.2:
            self.cmd_msg.angular.z = 0.65 / -1.177
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)

        # Maintain left turn for 1 second
        start_time_turn = rospy.Time.now()
        while (rospy.Time.now() - start_time_turn).to_sec() < 0.7:
            self.cmd_msg.angular.z = -1.1 / -1.177
            self.cmd_msg.linear.x = 0.885
            self.control.publish(self.cmd_msg)


    def lane_change(self):
        print("change lane to left")
        self.lane_change_l()
        
        start_time_straight = rospy.Time.now()
        while (rospy.Time.now() - start_time_straight).to_sec() < 0.2:
            print("LKAS")
            self.LKAS()

        print("change lane to right")
        self.lane_change_r()

    def stop(self):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 1:
            self.cmd_msg.angular.z = 0
            self.cmd_msg.linear.x = 0
            self.control.publish(self.cmd_msg)

    def end(self):
        self.cmd_msg.angular.z = 0
        self.cmd_msg.linear.x = 0
        self.control.publish(self.cmd_msg)


#### Action flag ####
# 0 => Naviagation
# 1 => LKAS
# 2 => Lane_change
# 3 => RoundAbout
# 4 => Intersection       
# 5 => turn left
# 6 => turn right
# 7 => go straight 
# 8 => Stop

def main():
    act = Action()
    try:
        while not rospy.is_shutdown():
            if act.state == 1:
                print("LKAS")
                act.LKAS()
            elif act.state == 2:
                print("lane chage") 
                act.lane_change()
                act.state = 1 # for activate LKAS after lane change
            elif act.state == 3:
                print("roundabout") 
                act.Roundabout()
                act.state = 1 # for activate LKAS after roundabout
            elif act.state == 4:
                print("intersection") 
                act.Intersection()
                act.state = 1 # for activate LKAS after intersection
            elif act.state == 5:
                print("turn left")                
                act.left()
                act.state = 1  # for activate LKAS after turn left
            elif act.state == 6:
                print("turn right") 
                act.right()
                act.state = 1  # for activate LKAS after turn right
            elif act.state == 7:
                print("go straight")
                act.straight()
                act.state = 1  # for activate LKAS after go straight
            elif act.state == 8:
                print("stop")
                act.end()
            elif act.state == 9:
                print("wait obstacle")
                act.stop()
                act.state = 1
            else:
                rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

