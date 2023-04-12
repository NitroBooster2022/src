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
from utils.msg import encoder, IMU

class EncoderNode():
    def __init__(self):
        print("init encoder node")
        rospy.init_node('encoder_node', anonymous=True)
        # self.image_sub = rospy.Subscriber("/automobile/encoder", Float32, self.callback, queue_size=3)
        self.twist_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback, queue_size=3)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.callbackI, queue_size=3)
        self.pub = rospy.Publisher("/automobile/encoder", encoder, queue_size = 3)
        self.p = encoder()
        self.rate = rospy.Rate(15)
        self.yaw = 0

    def callbackI(self, imu):
        self.yaw = imu.yaw #between pi to -pi

    def callback(self, ModelStates):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        header = Header()
        header.frame_id = 'encoder'
        header.stamp = rospy.Time.now()
        # header.seq = ModelStates.header.seq
        # header.stamp = ModelStates.header.stamp
        # header.frame_id = ModelStates.header.frame_id
        # Update the header information in the message
        self.p.header = header
        try:
            i = ModelStates.name.index("automobile") # index of the car
        except:
            return
        x_speed = ModelStates.twist[i].linear.x
        y_speed = ModelStates.twist[i].linear.y
        syaw = math.atan2(y_speed,x_speed)
        speed = math.sqrt(x_speed*x_speed+y_speed*y_speed)
        error = abs(syaw-self.yaw)
        if error>=5.73:
            error-=6.28
        if abs(error)<1.57:
            self.p.speed = speed
        else:
            self.p.speed = -speed
        # print("vx,vy,yaw,syaw",x_speed,y_speed,self.yaw,syaw)

        # t1 = time.time()

        # print(self.p)
        self.pub.publish(self.p)
        # print("time: ", time.time()-t1)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            node = EncoderNode()
            node.rate.sleep()
            rospy.spin()
        except rospy.ROSInterruptException:
            cv2.destroyAllWindows()
