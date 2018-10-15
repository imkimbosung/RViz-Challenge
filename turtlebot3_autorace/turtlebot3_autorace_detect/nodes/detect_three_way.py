# adding the Tree_way part
# -*- coding: utf-8 -*-


import rospy
import roslaunch
import numpy as np
import cv2
import subprocess
import os
import sys
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server
from turtlebot3_autorace_detect.cfg import DetectLaneParamsConfig
from enum import Enum
from std_msgs.msg import UInt8

class DetectThreeWay():
    def __init__(self):
       ## subscriber 
        # core_node_controller에서 받은 order
        self.sub_three_way_order = rospy.Subscriber(
            '/detect/three_way_order', UInt8, self.DecideOnDirection, queue_size=1)

        # detect_lane에서 받는order
      

       ## publisher 
        # core_node_controller 에  보내는 order
        self.pub_three_way_finished = rospy.Publisher(
            '/detect/three_way_finished', UInt8, queue_size=1)

        # detect_lane
        self.pub_detect_line = rospy.Publisher('/detect/three_way_line', UInt8, queue_size=1) 
        self.pub_detect_finish = rospy.Publisher('/detect/finish', UInt8, queue_size=1)

        self.StepOfThreeWay = Enum('StepOfThreeWay','go_to_left go_to_right pass_three_way')

    def DecideOnDirection(self,msg_pub_core):
        self.msg_decide = UInt8()
        self.msg_decide = msg_pub_core.data

        if self.StepOfThreeWay.go_to_left.value == self.msg_decide:
            print('send the left msg')
            self.pub_detect_line.publish(self.msg_decide)

        elif self.StepOfThreeWay.go_to_right.value == self.msg_decide:
              print('send the right msg')
              self.pub_detect_line.publish(self.msg_decide)
        elif self.StepOfThreeWay.pass_three_way.value == self.msg_decide:
              print('send the pass msg') 
              self.ResetToDetect(msg_decide)

    def ResetToDetect(self,msg):
        self.pub_detect_finish.publish(msg)


if __name__ == '__main__':
    rospy.init_node('detect_three_way')
    node = DetectThreeWay()
    node.main()
