#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert

import rospy
import numpy as np
import math
import os
import cv2
from enum import Enum
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

lower_blue = np.array([90,100,100])
upper_blue = np.array([130,255,255])
T_finish = 0

class DetectSign():
    def __init__(self):
	print("init")
        self.fnPreproc()

        self.sub_image_type = "raw" # you can choose image type "compressed", "raw"
        self.pub_image_type = "compressed" # you can choose image type "compressed", "raw"

        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.cbFindTrafficSign, queue_size = 1)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.cbFindTrafficSign, queue_size = 1)

        self.pub_traffic_sign = rospy.Publisher('/detect/traffic_sign', UInt8, queue_size=1)

        if self.pub_image_type == "compressed":
            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
        elif self.pub_image_type == "raw":
            # publishes traffic sign image in raw type
            self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output', Image, queue_size = 1)

        self.cvBridge = CvBridge()

        #change the order

        self.counter = 1
        self.number = 0

    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        #center_frame
        frame = cv_image_input[0:240,107:214]

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        median = cv2.medianBlur(gray,3)
        gray_blurred = cv2.GaussianBlur(median,(3,3),0)
        ret, threshold = cv2.threshold(gray_blurred,210,255,cv2.THRESH_BINARY)

        mask_blue = cv2.inRange(hsv,lower_blue,upper_blue)
        blue = cv2.bitwise_and(frame,frame,mask=mask_blue)

        blue_gray = cv2.cvtColor(blue,cv2.COLOR_BGR2GRAY)
        blue_gray_blurred = cv2.GaussianBlur(blue_gray,(5,5),0)

        ret_b, thresh_b = cv2.threshold(blue_gray_blurred,0,255,cv2.THRESH_BINARY)
        _, blue_contours, hierarchy = cv2.findContours(thresh_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, center_blue_contours, hierarchy = cv2.findContours(thresh_b[0:240,107,214], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        blue_x = []
        blue_y = []

        center_blue_x = []
        center_blue_y = []

        if T_finish == 0:
            for c in center_blue_contours:
                peri = cv2.arcLength(c,True)
                approx = cv2.approxPolyDP(c,0.04*peri,True)
                (x,y,w,h) = cv2.boundingRect(approx)
                end=x+w

                if w>20 and w<100 and h<60 and x<250 and end<280 and len(approx)!= 3 and len(approx)!= 6 :
                    center_blue_x.append(x)
                    center_blue_y.append(y)
                    cv2.drawContours(frame, [c], -1, (255,0,0),3)
            cv2.imshow('center_eye',frame)

        #nnnn>=5
        if len(center_blue_x)==2 and T_finish==0 and abs(center_blue_y[0]-center_blue_y[1])>70 and center_blue_y[0]*center_blue_y[1]==0 and abs(center_blue_x[0]-center_blue_x[1])<150 :
            if center_blue_x.index(max(center_blue_x)) == center_blue_y.index(max(center_blue_y)) :
                print("left")
            else :
                print("right")

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Threeway_exam')
    node = DetectSign()
    node.main()
