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
# from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# from pynput import keyboard
# from std_msgs.msg import String
from std_msgs.msg import Header
from utils.msg import Lane
# import scipy
# from message_filters import ApproximateTimeSynchronizer

class LaneDetector():
    def __init__(self, method = 'histogram', show=True):
        self.method = method
        self.show = show
        file = open(os.path.dirname(os.path.realpath(__file__))+'/houghlines.json', 'r')
        data = json.load(file)
        print("houghlines params:")
        print(data)
        # self.allKeys = ['=','-','w','s','r','l','g','p']
        self.point = alex.array(data.get('point'))
        self.res = data.get('res')
        self.threshold = data.get('threshold')
        self.minlength = data.get('minlength')
        self.error_p = alex.array(data.get('error_p'))
        self.error_w = data.get('error_w')
        self.stopline = False
        self.dotted = False
        self.pl = 320 # previous lane center
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_detector_node', anonymous=True)
        self.pub = rospy.Publisher("lane", Lane, queue_size=3)
        self.p = Lane()
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("automobile/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.image_callback)
        # self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)
        self.rate = rospy.Rate(15)

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        t1 = time.time()
         # Update the header information
        header = Header()
        header.seq = data.header.seq
        header.stamp = data.header.stamp
        header.frame_id = data.header.frame_id
        self.p.header = header

        # Convert the image to the OpenCV format
        # image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        image = self.bridge.imgmsg_to_cv2(data, "rgb8")

        #determine whether left lane is dotted
        # self.dotted = self.dotted_lines(image)
        # self.p.dotted = dotted

        # Extract the lanes from the image
        if self.method == 'histogram':
            lanes = self.histogram(image, show=self.show)
        else:
            lanes = self.extract_lanes(image, show=self.show)

        #Determine the steering angle based on the lanes

        # if there's a big shift in lane center: ignore due to delay
        if abs(lanes-self.pl)>250:
            lanes = self.pl

        # ignore one center measurement when we don't detect
        if lanes==320:
            self.p.center = self.pl
            self.pl = lanes
        else:
            self.p.center = lanes
            self.pl = lanes

        #determine whether we arrive at intersection
        self.p.stopline = self.stopline

        # print(self.p)
        # Publish the steering command
        self.pub.publish(self.p)
        # print(self.p)
        # print("time: ", time.time()-t1)

    def dotted_lines(self,image):
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = alex.zeros_like(img_gray)
        # poly = alex.array([[(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(int(1*w),int(0.8*h)),(w,h),(0,h)]])
        poly = alex.array([[(0,int(0.5*h)),(0,h),(int(0.4*w),h),(int(0.4*w),int(0.5*h))]]) # poly might need adjustment
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 150, 255, cv2.THRESH_BINARY) # threshold might need adjustment
        hist=alex.zeros((int(0.5*h),1))
        v=int(0.5*h)
        for i in range(int(0.5*h)):
            hist[i,0]=alex.sum(thresh[i+v,:])
        t = alex.mean(hist)
        lanes=[]
        p=0
        for i in range(int(0.5*h)):
            if hist[i,0]>t and p==0:
                lanes.append(i)
                p=t
            elif hist[i,0]<t/2 and p==t:
                lanes.append(i)
                p=0
        if len(lanes)%2==1:
            lanes.append(int(0.5*h)-1)
        if len(lanes)>5:
            return True
        else:
            return False

    def histogram(self, image, show=False):
        self.stopline = False
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = alex.zeros_like(img_gray)
        poly = alex.array([[(int(0*w),int(0.8*h)),(int(1*w),int(0.8*h)),(w,h),(0,h)]]) # poly might need adjustment
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 110, 255, cv2.THRESH_BINARY) # threshold might need adjustment
        
        centers=[]
        for i in range(int(w/2)):
            start=i
            end=w-i-1
            sumStart= alex.sum(thresh[:,start])
            sumEnd=alex.sum(thresh[:,end])
            flag1 = (sumStart>=1500)
            flag2 = (sumEnd>=1500)
            
            if (flag1 and flag2) and (abs(start-end)>3):
                centers.append((start+end)/2)
            
        # get lane centers based on 3 cases
        if len(centers) == 0: # no lane detected 
            center = w/2
            # case = 0
        elif len(centers) == 1: # one lane detected
            # case = 1
            if centers[0] > w/2:
                center = (centers[0]-0)/2
            else:
                center = (centers[0]*2+640)/2  
        elif abs(centers[0]-centers[len(centers)-1]) < 200: # the left most lane and the right most lane are close together (fuse them) 
            # case = 2
            center = (centers[0]+centers[len(centers)-1])/2
        if show:
            # print(hist[0,0:5])
            # print(lanes) 
            # print(centers)
            # cv2.putText(thresh, str(case), (int(w*0.9),int(h*0.1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
            if self.stopline == True:
                cv2.putText(thresh, 'stopline detected!', (int(w*0.1),int(h*0.1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
            if self.dotted == True:
                cv2.putText(image, 'DottedLine!', (int(w*0.1),int(h*0.3)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1, cv2.LINE_AA)
            cv2.line(thresh,(int(center),int(image.shape[0])),(int(center),int(0.8*image.shape[0])),(100,100,100),5)
            add = cv2.cvtColor(thresh,cv2.COLOR_GRAY2RGB)
            cv2.imshow('center', cv2.add(image,add))
            cv2.waitKey(1)
        return center

    def extract_lanes(self, image, show=False): #hough transform
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
        vertices = alex.array([[(0,h*0.75),(self.point[0]*w,self.point[1]*h),(w,0.75*h),(w,h),(0,h)]], dtype=alex.int32)
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
                    if abs(x1-x2)>w/10 and y1>0.8*h:
                        # print(x1,y1,x2,y2)
                        # print(len(lines))
                        self.stopline = True
        if len(left) == 0:
            left_lane = 0
        else:
            left_lane = alex.mean(left)
        if len(right) == 0:
            right_lane = w
        else:
            right_lane = alex.mean(right)
        # print("time used: ", time.time()-t1)
        center = (left_lane+right_lane)/2
        if show:
            cv2.line(image,(int(center),int(image.shape[0])),(int(center),int(0.8*image.shape[0])),(255,0,255),5)
            cv2.imshow('center', image)
            cv2.waitKey(1)
        return (left_lane+right_lane)/2

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=str, default='histogram', help="hough or histogram")
    parser.add_argument("--show", type=str, default=True, help="show camera frames")
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        if args.show=="True":
            s = True
        else:
            s = False
        node = LaneDetector(method=args.method, show = s)
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()