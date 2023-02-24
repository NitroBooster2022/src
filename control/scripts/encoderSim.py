#!/usr/bin/env python3

# import onnxruntime
# from yolov7 import YOLOv7
import argparse
import rospy
import json
import cv2
import os
import time
import numpy as alex
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# from pynput import keyboard
from std_msgs.msg import String, Float32
from std_msgs.msg import Header
from message_filters import ApproximateTimeSynchronizer
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
import math
from utils.msg import Encoder

class EncoderNode():
    def __init__(self):
        print("init encoder node")
        rospy.init_node('encoder_node', anonymous=True)
        # self.image_sub = rospy.Subscriber("/automobile/encoder", Float32, self.callback, queue_size=3)
        self.image_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback, queue_size=3)
        self.pub = rospy.Publisher("encoder", encoder, queue_size = 3)
        self.p = encoder()
        self.rate = rospy.Rate(15)

    def callback(self, ModelStates):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """

        header = Header()
        print("hi")
        header.frame_id = 'encoder'
        header.stamp = rospy.Time.now()
        # header.seq = ModelStates.header.seq
        # header.stamp = ModelStates.header.stamp
        # header.frame_id = ModelStates.header.frame_id
        # Update the header information in the message
        self.p.header = header
        x_speed = ModelStates.twist[72].linear.x
        y_speed = ModelStates.twist[72].linear.y
        speed = math.sqrt(x_speed*x_speed+y_speed*y_speed)
        self.p.speed = speed
        t1 = time.time()


        print(self.p)
        self.pub.publish(self.p)
        # print("time: ", time.time()-t1)

if __name__ == '__main__':
    try:
        node = EncoderNode()
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
