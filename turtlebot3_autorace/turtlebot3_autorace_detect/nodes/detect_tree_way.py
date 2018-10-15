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

        # core_node_controller에서 받은 order
        self.sub_three_way_order = rospy.Subscriber(
            '/detect/three_way_order', UInt8, self.DecideOnDirection, queue_size=1)

        # detect_lane에서 받는order

        # core_node_controller 에  보내는 order
        self.pub_three_way_finished = rospy.Publisher(
            '/detect/three_way_finished', UInt8, queue_size=1)

        # detect_lane에

        self.StepOfTreeWay = Enum('StepOfTreeWay','

    def DecideOnDirection(self,msg_pub_core):
        
