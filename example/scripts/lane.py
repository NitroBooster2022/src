#!/usr/bin/env python3

# import onnxruntime
# from yolov7 import YOLOv7
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
# from control.msg import Lane
from message_filters import ApproximateTimeSynchronizer

class LaneDetector():
    def __init__(self):
        file = open(os.path.dirname(os.path.realpath(__file__))+'/json-dump.json', 'r')
        data = json.load(file)
        print(data)
        self.allKeys = ['=','-','w','s','r','l','g','p']
        self.point = alex.array(data.get('point'))
        self.res = data.get('res')
        self.threshold = data.get('threshold')
        self.minlength = data.get('minlength')
        self.error_p = alex.array(data.get('error_p'))
        self.error_w = data.get('error_w')
        self.p = 0.006
        self.i = 0
        self.d = 0.003
        self.last = 0
        self.stopline = False
        self.inter_dec = 'stop'
        self.maxspeed = 0.2
        self.inter_dec = ''
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_detector_node', anonymous=True)
        # self.pub = rospy.Publisher("lane", Lane, queue_size=2)
        # self.p = Lane()
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)
        # self.image_sync = ApproximateTimeSynchronizer([self.image_sub], queue_size = 2, slop=0.1)
        # # Register the image_callback function to be called when a synchronized message is received
        # self.image_sync.registerCallback(self.image_callback)
        self.rate = rospy.Rate(20)

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        # Convert the image to the OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Frame preview1", image)
        key = cv2.waitKey(1)
        # Extract the lanes from the image
        lanes = self.extract_lanes(image)

        #Determine the steering angle based on the lanes
        #print("lane following")
        # self.p.center = lanes#
        # # Publish the steering command
        # self.pub.publish(self.p)

    def extract_lanes(self, image):
        """
        Extract the lanes from the image
        :param image: Image data in the OpenCV format
        :return: Tuple containing the left and right lanes (each a list of points)
        """
        # Convert the image to grayscale and apply a blur to remove high frequency noise
        # Apply a Canny edge detector to find the edges in the image
        t1 = time.time()
        edges = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),(5,5),0),50,150)

        # Create a mask for the region of interest (ROI) in the image where the lanes are likely to be
        mask = alex.zeros_like(edges)
        h = image.shape[0]
        w = image.shape[1]
        vertices = alex.array([[(0,h*0.8),(self.point[0]*w,self.point[1]*h),(w,0.8*h),(w,h),(0,h)]], dtype=alex.int32)
        cv2.fillPoly(mask, vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Use Hough transform to detect lines in the image
        lines = cv2.HoughLinesP(masked_edges,self.res,alex.pi/180,self.threshold,minLineLength=self.minlength)

        # Separate the lines into left and right lanes
        left=[]
        right=[]
        self.stopline = False
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line.reshape(4)
                if (x2-x1)==0 or abs((y2-y1)/(x2-x1)) > 0.1:
                    m = (x2-x1)/(y2-y1)
                    p_y = int(self.error_p[1]*h)
                    p_x = int(x1 + (p_y-y1)*m)
                    if p_x < w/2:
                        if p_x < int((self.error_p[0]+self.error_w)*w) and p_x > int(self.error_p[0]*w):
                            left.append(p_x)
                    else:
                        if p_x < int((1-self.error_p[0])*w) and p_x > int((1-self.error_p[0]-self.error_w)*w):
                            right.append(p_x)
                else:
                    if abs(x1-x2)>w/10 and y1>0.85*h:
                        print(x1,y1,x2,y2)
                        print(len(lines))
                        self.stopline = True
        if len(left) == 0:
            left_lane = 0
        else:
            left_lane = alex.mean(left)
        if len(right) == 0:
            right_lane = w
        else:
            right_lane = alex.mean(right)
        print("time used: ", time.time()-t1)
        print((left_lane+right_lane)/2)
        return (left_lane+right_lane)/2

if __name__ == '__main__':
    try:
        node = LaneDetector()
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

