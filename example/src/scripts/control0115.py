#!/usr/bin/env python3

import onnxruntime
from yolov7 import YOLOv7
import rospy
import json
import cv2
import os
import time
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from pynput import keyboard
from std_msgs.msg import String, Byte
from srv import *

class LaneFollower():
    def __init__(self):

        # sign detector
        model_path = os.path.dirname(os.path.realpath(__file__))+"/models/s12.onnx"
        # model_path = os.path.dirname(os.path.realpath(__file__))+"/models/best11c.onnx"
        self.yolov7_detector = YOLOv7(model_path, conf_thres=0.75, iou_thres=0.57)
        self.class_ids=[]
        self.boxes=[]
        self.scores = []
        self.class_names = ['oneway', 'highwayexit', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayentrance', 'priority',
                'lights','block','pedestrian','car','others','others','others','others','others','others',
                'others','others','others','others','others','others','others','others','others','others',
                'others','others','others','others','others','others','others','others','others','others',
                'others','others','others','others','others','others','others','others','others','others',
                'others','others','others','others','others','others','others','others','others','others',
                'others','others','others','others','others','others','others','others','others','others',]
        self.stop_sizes = [50,50,55,50,50,65,50,50,60,25,50,100,100]
        self.detected_id = 0

        # loading hough line parameters
        file = open(os.path.dirname(os.path.realpath(__file__))+'/json-dump.json', 'r')
        data = json.load(file)
        print(data)
        
        self.point = np.array(data.get('point'))
        self.res = data.get('res')
        self.threshold = data.get('threshold')
        self.minlength = data.get('minlength')
        self.error_p = np.array(data.get('error_p'))
        self.error_w = data.get('error_w')
        self.p = 0.006
        self.stopline = False
        self.inter_dec = 'straight'
        self.maxspeed = 0.15
        self.i = 0
        self.d = 0.003
        self.last = 0
        self.center = 0
        self.leftlane = 0
        self.rightlane = 640
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_follower_node', anonymous=True)
        self.get_dir = rospy.ServiceProxy('get_direction',get_direction)
        rospy.wait_for_service('get_direction')
        self.cd = rospy.Time.now()
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=1)
        # self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)
        self.rate = rospy.Rate(10)
        # keyboard listener
        # self.keyInput()

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        # Convert the image to the OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

        # cv2.imshow("Frame", image)
        # key = cv2.waitKey(1)

        # object detection
        # self.detect_object(image)

        # parking sign decision
        if self.detected_id==4:
            print('parking detected')
            self.park()
            self.detected_id=0

        # block decision
        elif self.detected_id==10:
            print('roadblock detected')
            if not(self.dotline()):
                return
            self.takeover()
            self.detected_id=0
            return

        # pedestrian decision
        elif self.detected_id==11:
            print('pedestrian detected')
            self.stop(3)
            self.detected_id=0
            return

        # car decision
        elif self.detected_id==12:
            print('car')
            if not(self.dotline()):
                return
            self.takeover()
            self.detected_id=0
            return

        # stopline decision
        if self.stopline and rospy.Time.now()>=self.cd:
            print('decision')
            # traffic light decision
            if self.detected_id==9:
                self.stop(1)
                print('traffic light detected')
                # get the position fromm service to determine which traffic light
                r=self.get_dir(0,0,0,'').dir
                if r.split()[2]=='N' or r.split()[2]=='S':
                    topic = 'start'
                else:
                    topic = 'master'
                state=rospy.wait_for_message('/automobile/trafficlight/'+topic,Byte)
                if state.data==0:
                    print('redlight')
                    self.stop(1)
                    self.detected_id=0
                    return
                elif state.data==1:
                    print('yellowlight')
                    self.stop(1)
                    self.detected_id=0
                    return
                else:
                    print('greenlight')
                    self.detected_id=0
            self.intersection_decision()
            self.stopline = False
            self.cd = rospy.Time.now()+rospy.Duration(1.5)
        
        # Extract the lane center from the image
        center = self.histogram(image)

        # Determine the steering angle based on the center
        steering_angle = self.get_steering_angle(center)

        # Publish the steering command
        self.publish_cmd_vel(steering_angle)

    def intersection_decision(self):
        """
        Decision process
        """
        self.stop(0.4)
        self.maxspeed = 0.15

        #if stopsign is detected, stop at stopline
        if self.detected_id==2:
            print('stopsign detected')
            rospy.loginfo('stopping for 3 seconds')
            self.stop(3)
            self.detected_id=0

        #if pedestrian crosswalk is detected, slowdown
        if self.detected_id==5:
            print('pedestrian crosswalk detected')
            rospy.loginfo('slowdown')
            self.maxspeed = 0.05
            self.detected_id=0
        
        # get decision from map
        r=self.get_dir(0,0,0,'').dir
        self.inter_dec=r.split()[0]
        # dest = r.split()[3]

        # adjustment steering at stopline
        offset = float(r.split()[1])
        self.maxspeed = 0.05
        while abs(offset) > 0.03:
            self.publish_cmd_vel(offset)
            offset = float(self.get_dir(0,0,0,'').dir.split()[1])
        self.maxspeed = 0.15

        # self.go_back(0.5)
        
        rospy.loginfo("intersection detected, current decision: %s", self.inter_dec)
        if self.inter_dec=='left':
            rospy.loginfo('turn left')
            self.go_straight(3.5)
            self.turn_left(8)
        elif self.inter_dec=='right':
            rospy.loginfo('turn right')
            self.go_straight(1.5)
            self.turn_right(8)
        elif self.inter_dec=='straight':
            rospy.loginfo('go straight')
            self.go_straight(0.3)
        elif self.inter_dec=='idle':
            rospy.loginfo('idle')
            rospy.loginfo('destination reached')
            self.idle()

    def stop(self,t):
        """
        Stop the car for t seconds
        :param t: stop time
        """
        rospy.loginfo("stop function called")
        msg = String()
        msg.data = '{"action":"3","steerAngle":'+str(0)+'}'
        self.cmd_vel_pub.publish(msg)
        end_time = rospy.Time.now()+rospy.Duration(t)
        while rospy.Time.now()<end_time:
            self.rate.sleep()

    def go_straight(self,t):
        """
        Go straight
        :param t: travel time
        """
        rospy.loginfo("go straight called %.2f seconds", t)
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(0.2)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(0)+'}'
        t_end = rospy.Time.now()+rospy.Duration(t)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg2)
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        self.cmd_vel_pub.publish(msg)

    def go_back(self,t):
        """
        Go straight
        :param t: travel time
        """
        # rospy.loginfo("go back called %.2f seconds", t)
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(-0.2)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(0)+'}'
        t_end = rospy.Time.now()+rospy.Duration(t)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg2)
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        self.cmd_vel_pub.publish(msg)

    def idle(self):
        """
        Stop the car till not idle
        :param t: stop time
        """
        msg = String()
        while True:
            msg.data = '{"action":"3","steerAngle":'+str(0)+'}'
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

    def turn_left(self,t):
        """
        Turn left
        :param t: turn time
        """
        self.intersection_detected = True
        self.inter_dec = 'left'
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(0.12)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(-23)+'}'
        t_end = rospy.Time.now() + rospy.Duration(t)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_pub.publish(msg2)
            self.rate.sleep()

    def turn_right(self,t):
        """
        Turn right
        :param t: turn time
        """
        self.inter_dec = 'right'
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(0.12)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(23)+'}'
        t_end = rospy.Time.now() + rospy.Duration(t)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_pub.publish(msg2)

    def park(self):
        r=self.get_dir(0,0,0,'').dir
        if r.split()[2]=='E':
            rospy.loginfo('parkN')
            self.go_straight(3.5)
            self.turn_right(8)
        else:
            rospy.loginfo('parkS')
            self.go_straight(2.5)
            self.turn_right(2)
            self.go_straight(1.5)
            self.turn_left(2)
        rospy.loginfo('destination reached')
        self.image_sub.unregister()
        self.idle()

    def takeover(self):
        self.turn_left(3.5)
        self.go_straight(0.5)
        self.turn_right(3.5)
        self.go_straight(2.5)
        self.turn_right(3.5)
        self.go_straight(0.5)
        self.turn_left(3.5)

    def detect_object(self, image):
        # object detector
        self.boxes, self.scores, self.class_ids = self.yolov7_detector(image.copy())

        # check detected objects
        if any(self.class_ids) and rospy.Time.now()>=self.cd: #not empty
            id = self.class_ids[0]
            if id<=12:
                # if the object is big enough, consider it in range
                box_size = max([(self.boxes[0][2]-self.boxes[0][0]),(self.boxes[0][3]-self.boxes[0][1])])
                if box_size >= self.stop_sizes[self.class_ids[0]]:
                    if self.detected_id != id:
                        self.detected_id = id
                        print(self.class_names[self.detected_id]+" detected!!, size is "+str(box_size))

    def houghlines(self, image):
        """
        Extract the lanes from the image
        :param image: Image data in the OpenCV format
        :return: Tuple containing the left and right lanes (each a list of points)
        """
        # Convert the image to grayscale and apply a blur to remove high frequency noise
        # Apply a Canny edge detector to find the edges in the image
        edges = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),(5,5),0),50,150)

        # Create a mask for the region of interest (ROI) in the image where the lanes are likely to be
        mask = np.zeros_like(edges)
        h = image.shape[0]
        w = image.shape[1]
        vertices = np.array([[(0,h*0.8),(self.point[0]*w,self.point[1]*h),(w,0.8*h),(w,h),(0,h)]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Use Hough transform to detect lines in the image
        lines = cv2.HoughLinesP(masked_edges,self.res,np.pi/180,self.threshold,minLineLength=self.minlength)

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
                        self.stopline = True
        if len(left) == 0:
            left_lane = 0
        else:
            left_lane = np.mean(left)
        if len(right) == 0:
            right_lane = w
        else:
            right_lane = np.mean(right)
        self.leftlane = left_lane
        self.rightlane = right_lane
        return (left_lane+right_lane)/2

    def histogram(self, image):
        self.stopline = False
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = np.zeros_like(img_gray)
        # poly = np.array([[(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(int(1*w),int(0.8*h)),(w,h),(0,h)]])
        poly = np.array([[(int(0*w),int(0.85*h)),(int(1*w),int(0.85*h)),(w,h),(0,h)]])
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 150, 255, cv2.THRESH_BINARY)
        hist=np.zeros((1,w))
        for i in range(w):
            hist[0,i]=np.sum(thresh[:,i])
        lanes=[]
        p=0
        for i in range(w):
            if hist[0,i]==255 and p==0:
                lanes.append(i)
                p=255
            elif hist[0,i]==0 and p==255:
                lanes.append(i)
                p=0
        if len(lanes)%2==1:
            lanes.append(w-1)
        centers=[]
        for i in range(int(len(lanes)/2)):
            if abs(lanes[2*i]-lanes[2*i+1])>350:
                self.stopline = True
            elif abs(lanes[2*i]-lanes[2*i+1])>3:
                centers.append((lanes[2*i]+lanes[2*i+1])/2)
        if len(centers)==0:
            # no lane detected
            self.leftlane = -1
            self.rightlane = w
            return w/2
        elif len(centers)==1:
            # one lane detected
            if centers[0]>w/2:
                # rightlane
                self.leftlane = -1
                self.rightlane = centers[0]
                return (centers[0]+0)/2
            else:
                # leftlane
                self.leftlane = centers[0]
                self.rightlane = w
                return (centers[0]+w)/2
        elif abs(centers[len(centers)-1]-centers[0])<200:
            # group lanes together
            if (centers[len(centers)-1]+centers[0])>w:
                self.leftlane = -1
                self.rightlane = (centers[len(centers)-1]+centers[0])/2
                return ((centers[len(centers)-1]+centers[0])/2+0)/2
            else:
                self.leftlane = (centers[len(centers)-1]+centers[0])/2
                self.rightlane = w
                return ((centers[len(centers)-1]+centers[0])/2+w)/2
        else:
            # rightlane - leftlane
            self.leftlane = centers[0]
            self.rightlane = centers[len(centers)-1]
            return (centers[len(centers)-1]+centers[0])/2

    def dotted_lines(self,image):
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = np.zeros_like(img_gray)
        # poly = np.array([[(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(int(1*w),int(0.8*h)),(w,h),(0,h)]])
        poly = np.array([[(0,int(0.5*h)),(0,h),(int(0.4*w),h),(int(0.4*w),int(0.5*h))]])
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 150, 255, cv2.THRESH_BINARY)
        hist=np.zeros((int(0.5*h),1))
        v=int(0.5*h)
        for i in range(int(0.5*h)):
            hist[i,0]=np.sum(thresh[i+v,:])
        t = np.mean(hist)
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

    def get_steering_angle(self, center):
        """
        Determine the steering angle based on the lane center
        :param center: lane center
        :return: Steering angle in radians
        """

        # roundabout
        # if self.detected_id == 3:
        # if True:
            # follow leftlane
            # if self.leftlane < 0:
            #     return -0.4
            # error = (self.leftlane-100)
            # d_error = error-self.last
            # self.last = error
            # steering_angle = (error * self.p+d_error*self.d)
            # steering_angle = np.clip(steering_angle, -0.4, 0.4)
            # return steering_angle

            # follow rightlane
            # if self.rightlane > 640:
            #     return 0.4
            # error = (self.rightlane-600)
            # d_error = error-self.last
            # self.last = error
            # steering_angle = (error * self.p+d_error*self.d)
            # steering_angle = np.clip(steering_angle, -0.4, 0.4)
            # return steering_angle
        # Calculate the steering angle
        image_center = 640 / 2

        if center==image_center:
            center=self.center
            self.center=image_center
        else:
            self.center=center
        
        error = (center - image_center)
        d_error = error-self.last
        self.last = error
        steering_angle = (error * self.p+d_error*self.d)
        steering_angle = np.clip(steering_angle, -0.4, 0.4)

        return steering_angle

    def publish_cmd_vel(self, steering_angle):
        """
        Publish the steering command to the cmd_vel topic
        :param steering_angle: Steering angle in radians
        """
        msg = String()
        msg2 = String()
        x = self.maxspeed + self.maxspeed*abs(steering_angle)/0.4
        msg.data = '{"action":"1","speed":'+str(x)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(steering_angle*180/np.pi)+'}'
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_pub.publish(msg2)
        # rospy.loginfo("published data: %s", msg.data)

    def keyInput(self):
        self.allKeys = ['=','-','w','s','r','l','g','p','e']
        with keyboard.Listener(on_press = self.keyPress) as listener:
            listener.join()

    # ===================================== KEY PRESS ====================================
    def keyPress(self,key):
        """Processing the key pressing 
        
        Parameters
        ----------
        key : pynput.keyboard.Key
            The key pressed
        """                                     
        try:
            if key.char in self.allKeys:
                if key.char == '=':
                    self.p += 0.001
                elif key.char == '-':
                    self.p -= 0.001
                elif key.char == 'w':
                    self.maxspeed += 0.01
                elif key.char == 'e':
                    self.maxspeed -= 0.01
                elif key.char == 'r':
                    self.inter_dec = 'right'
                    print(self.inter_dec)
                elif key.char == 'l':
                    self.inter_dec = 'left'
                    print(self.inter_dec)
                elif key.char == 's':
                    self.inter_dec = 'straight'
                    print(self.inter_dec)
                elif key.char == 'i':
                    self.inter_dec = 'idle'
                    print(self.inter_dec)
                elif key.char == 'g':
                    self.inter_dec = ''
                elif key.char == 'p':
                    print('p,maxspeed',self.p,self.maxspeed)
        except: pass

if __name__ == '__main__':
    try:
        node = LaneFollower()
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
