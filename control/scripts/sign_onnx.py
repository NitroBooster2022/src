#!/usr/bin/env python3

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
from std_msgs.msg import Header
from utils.msg import Sign
import onnxruntime
from yolov7 import YOLOv7
from yolov7.utils import draw_detections


class ObjectDetector():
    def __init__(self, show):
        self.show = show
        self.model = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "models/alex12s2.onnx")
        self.yolov7_detector = YOLOv7(self.model, conf_thres=0.75, iou_thres=0.57)
        # self.net = cv2.dnn.readNet(self.model)
        self.class_names = ['oneway', 'highwayexit', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayentrance', 'priority', 'light', 'block', 'girl', 'car']
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)
        self.pub = rospy.Publisher("sign", Sign, queue_size = 3)
        self.p = Sign()
        self.rate = rospy.Rate(15)

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        t1 = time.time()
        # Convert the image to the OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

         # Update the header information
        header = Header()
        header.seq = data.header.seq
        header.stamp = data.header.stamp
        header.frame_id = data.header.frame_id
        # Update the header information in the message
        self.p.header = header

        # self.class_ids, __, self.boxes = self.detect(image, self.class_list, show=self.show)
        self.boxes, self.scores, self.class_ids = self.yolov7_detector(image)
        if self.show:
            img = draw_detections(image, self.boxes, self.scores, self.class_ids)
            # cv2.rectangle(image, (100, 100), (200, 300), (255,0,0), 2)
            cv2.imshow('sign', img)
            cv2.waitKey(3)
        self.p.objects = self.class_ids
        self.p.num = len(self.class_ids)
        if self.p.num>=2:
            height1 = self.boxes[0][3]-self.boxes[0][1]
            width1 = self.boxes[0][2]-self.boxes[0][0]
            self.boxes[0][2] = width1
            self.boxes[0][3] = height1
            height2 = self.boxes[1][3]-self.boxes[1][1]
            width2 = self.boxes[1][2]-self.boxes[1][0]
            self.boxes[1][2] = width2
            self.boxes[1][3] = height2
            self.p.box1 = self.boxes[0]
            self.p.box2 = self.boxes[1]
        elif self.p.num>=1:
            height1 = self.boxes[0][3]-self.boxes[0][1]
            width1 = self.boxes[0][2]-self.boxes[0][0]
            self.boxes[0][2] = width1
            self.boxes[0][3] = height1
            self.p.box1 = self.boxes[0]

        print(self.p)
        self.pub.publish(self.p)
        print("time: ",time.time()-t1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--show", type=str, default=False, help="show camera frames")
    args = parser.parse_args()
    try:
        node = ObjectDetector(show = args.show)
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
