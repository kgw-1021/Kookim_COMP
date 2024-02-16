#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
from math import *
import os

class Bike_bus_stop:
    def __init__(self) :
        rospy.init_node("bike_bus_stop")
        self.state = 0
        rospy.Subscriber("/recog_flag", Float64, self.state_CB) 
        rospy.Subscriber("/image_jpeg/compressed",CompressedImage,callback = self.cam_CB)
        rospy.Subscriber("/scan",LaserScan,callback = self.lidar_CB)
        # self.img_pub = rospy.Publisher("/cam/compressed",CompressedImage,queue_size=10)
        self.obstacle_pub = rospy.Publisher("/obstacle_flag",Float64,queue_size=1)
        self.obstacle_flag = 0
        self.image_msg = CompressedImage()
        self.scan_msg = LaserScan()
        self.bridge = CvBridge()
        self.speed_msg = Float64()
        self.bike_flage = False
        self.bus_flage = False
        self.lidar_flage = False
        self.img2 = cv2.imread('_bike.png', 0)
        self.img3 = cv2.imread('_bus1.png', 0)
        self.img4 = cv2.imread('_bus2.png', 0)
        self.img2 = self.img2[124:360,160:470]
        self.img3 = self.img3[120:360,160:450]
        self.img4 = self.img4[120:360,160:450]
        # ORB 검출기 초기화
        self.orb = cv2.ORB_create()
        self.kp2, self.des2 = self.orb.detectAndCompute(self.img2, None)
        self.kp3, self.des3 = self.orb.detectAndCompute(self.img3, None)
        self.kp4, self.des4 = self.orb.detectAndCompute(self.img4, None)

    def state_CB(self, data):
        self.state = data.data


    def lidar_CB(self,msg):
        if self.state == 1:
            self.scan_msg = msg
            self.lidar_flage = self.lidar()

    def lidar(self):
        degree_min = self.scan_msg.angle_min * 180/pi
        degree_max = self.scan_msg.angle_max * 180/pi
        degree_anle_increment = self.scan_msg.angle_increment * 180/pi

        # print(self.scan_msg)
        # print(degree_min)
        # print(degree_max)
        # print(degree_anle_increment)
        # print(len(self.scan_msg.ranges))
        # self.scan_msg.ranges -> 범위 값[m]

        degrees = [degree_min + degree_anle_increment * index for index, value in enumerate(self.scan_msg.ranges)] # 각도 값[도]
        obstacle_degrees = []
        for index , value in enumerate(self.scan_msg.ranges):
            if (-11 < degrees[index] < 11) and 0 < value < 1.2: # 각도 값, 거리 값 함께 고려
                # print(f"{index} : {degrees[index]}") # 이쪽 인덱스, 각도에 장애물
                obstacle_degrees.append(degrees[index])
            else:
                pass
        
        # print(len(obstacle_degrees))
        # print(obstacle_degrees)
        if len(obstacle_degrees) > 0:
            self.lidar_flage = True
        else:
            self.lidar_flage = False    
        
        return self.lidar_flage

    def cam_CB(self,msg):
        if self.state == 1:
            self.image_msg = msg
            self.img = self.bridge.compressed_imgmsg_to_cv2(self.image_msg)
            self.bike_flage, self.bus_flage = self.bike_or_bus()
            if self.bike_flage == True and self.lidar_flage == True:
                self.obstacle_flag = 2
                print("Bike")
            elif self.bus_flage == True and self.lidar_flage == True:
                self.obstacle_flag = 1
                print("Bus")
            else:
                self.obstacle_flag = 0
                print("no_obstacle")
            self.obstacle_pub.publish(self.obstacle_flag)
        else: 
            rospy.sleep(1)

    def bike_or_bus(self):
        # 이미지 불러오기
        img1 = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        img1 = img1[140:340,180:460]
        # tmp = self.bridge.cv2_to_compressed_imgmsg(img1)
        # self.img_pub.publish(tmp)
        # ORB로 키포인트와 기술자 찾기
        kp1, des1 = self.orb.detectAndCompute(img1, None)
        
        # BFMatcher (Brute Force Matcher) 생성
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # 기술자 매칭
        matches_bike = bf.match(des1, self.des2)
        matches_bus = bf.match(des1, self.des3)
        matches_bus_b = bf.match(des1, self.des4)

        # 거리가 50 미만인 매칭만 사용하여 분류
        filtered_matches_bike = [match for match in matches_bike if match.distance < 48]
        filtered_matches_bus = [match for match in matches_bus if match.distance < 47]
        filtered_matches_bus_b = [match for match in matches_bus_b if match.distance < 47]

        # 분류된 매칭을 그림
        # img_matches_bike = cv2.drawMatches(img1, kp1, self.img2, self.kp2, filtered_matches_bike, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        # img_matches_bus = cv2.drawMatches(img1, kp1, self.img3, self.kp3, filtered_matches_bus, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        # img_matches_bus_b = cv2.drawMatches(img1, kp1, self.img4, self.kp4, filtered_matches_bus_b, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        # # img_matches_bike와 img_matches_bus의 크기를 맞춰야 함
        # min_height = min(img_matches_bike.shape[0], img_matches_bus.shape[0],img_matches_bus_b.shape[0])
        # min_width = min(img_matches_bike.shape[1],img_matches_bus.shape[1],img_matches_bus_b.shape[1])
        # img_matches_bike = img_matches_bike[:min_height, :min_width]
        # img_matches_bus = img_matches_bus[:min_height, :min_width]
        # img_matches_bus_b = img_matches_bus_b[:min_height, :min_width]
        # img_matches = cv2.vconcat( [img_matches_bike, img_matches_bus, img_matches_bus_b] )

        # tmp = self.bridge.cv2_to_compressed_imgmsg(img_matches)
        # self.img_pub.publish(tmp)


        # 거리가 50 미만인 매칭만을 사용하여 분류한 경우
        if len(filtered_matches_bike) > 4:  # 예를 들어, 5개 이상의 매칭이 있다면 분류된 이미지로 판단할 수 있음
            self.bike_flage = True
            # print("Yes bike")
        
        elif not self.bike_flage == True and len(filtered_matches_bus) > 8 or len(filtered_matches_bus_b) > 8:  # 예를 들어, 5개 이상의 매칭이 있다면 분류된 이미지로 판단할 수 있음
            self.bus_flage = True
            # print("Yes bus")
        else:
            self.bus_flage = False
            self.bike_flage = False
            # print("No bus")

        return self.bike_flage, self.bus_flage

        

if __name__=='__main__':
    bike_bus_stop = Bike_bus_stop()
    rospy.sleep(3)
    rospy.spin()
