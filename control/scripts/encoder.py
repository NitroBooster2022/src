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
from std_msgs.msg import String
from std_msgs.msg import Header
from utils.msg import Encoder
from message_filters import ApproximateTimeSynchronizer
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
import math

class EncoderNode():
    def __init__(self):
        print("init encoder node")
        rospy.init_node('encoder_node', anonymous=True)
        self.image_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, queue_size=3)
        # self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)
        self.pub = rospy.Publisher("encoder", Encoder, queue_size = 3)
        self.p = Encoder()
        self.rate = rospy.Rate(15)

    def image_callback(self, ModelStates):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        header = Header()
        print("hi")
        header.seq = ModelStates.header.seq
        header.stamp = ModelStates.header.stamp
        header.frame_id = ModelStates.header.frame_id
        # Update the header information in the message
        self.p.header = header
        x_speed = ModelStates.twist[72].linear.x
        y_speed = ModelStates.twist[72].linear.y
        speed = math.sqrt(x_speed*x_speed+y_speed*y_speed)
        self.p.speed = speed
        t1 = time.time()

        print(self.p)
        self.pub.publish(self.p)
        print("time: ", time.time()-t1)

    

if __name__ == '__main__':
    try:
        node = EncoderNode()
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()