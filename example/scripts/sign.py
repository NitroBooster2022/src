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
from control.msg import Sign
from message_filters import ApproximateTimeSynchronizer

def format_yolov5(frame):
    row, col, _ = frame.shape
    _max = max(col, row)
    result = alex.zeros((_max, _max, 3), alex.uint8)
    result[0:row, 0:col] = frame
    return result

class ObjectDetector():
    def __init__(self):
        self.net = cv2.dnn.readNet('/home/simonli/Documents/Simulator/src/control/models/alex12s2.onnx')
        self.class_list = ['oneway', 'highwayexit', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayentrance', 'priority', 'light', 'block', 'girl', 'car']
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)
        # self.image_sync = ApproximateTimeSynchronizer([self.image_sub], queue_size = 2, slop=0.1)
        # # Register the image_callback function to be called when a synchronized message is received
        # self.image_sync.registerCallback(self.image_callback)
        self.pub = rospy.Publisher("detected_objects", Sign, queue_size = 3)
        self.p = Sign()
        self.rate = rospy.Rate(20)

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        # Convert the image to the OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Frame preview", image)
        key = cv2.waitKey(1)
        self.boxes, self.scores, self.class_ids = self.yolov7_detector(image)
        class_ids, confidences, boxes, t2 = self.detect(image, self.class_list)
        if len(class_ids)>=2:
            self.p.obj2 = class_ids[1]
            self.p.left2 = boxes[1][0]
            self.p.top2 = boxes[1][1]
            self.p.width2 = boxes[1][2]
            self.p.height2 = boxes[1][3]
            self.p.obj1 = class_ids[0]
            self.p.left1 = boxes[0][0]
            self.p.top1 = boxes[0][1]
            self.p.width1 = boxes[0][2]
            self.p.height1 = boxes[0][3]
        elif len(class_ids)>=1:
            self.p.obj2 = -1
            self.p.obj1 = class_ids[0]
            self.p.left1 = boxes[0][0]
            self.p.top1 = boxes[0][1]
            self.p.width1 = boxes[0][2]
            self.p.height1 = boxes[0][3]
        else:
            self.p.obj1 = -1
            self.p.obj2 = -1
        self.pub.publish(self.p)

        # #print("inference time: ", time.time()-t1)
        # print("ids: ",self.class_ids)
        # if any(self.class_ids) and rospy.Time.now()>=self.cd: #not empty
        #     id = self.class_ids[0]
        #     if id == 2 or id==5: #stopsign or crosswalk
        #         box = self.boxes[0] 
        #         box_area = (box[2] - box[0]) * (box[3] - box[1])
        #         print("box area: ", box_area)
        #         if box_area >= self.stop_sizes[id]:
        #             name = self.class_names[id]
        #             print(name+" detected!!, size is " + str(box_area))
        #             self.stop(3)
        #             self.intersection_decision()
        #             self.cd = rospy.Time.now()+rospy.Duration(1.5)
        self.pub.publish(self.p)
    def detect(self, image, class_list, save=False, show=False):
        t1 = time.time()
        input_image = format_yolov5(image) # making the image square
        blob = cv2.dnn.blobFromImage(input_image , 1/255.0, (640, 640), swapRB=True)
        self.net.setInput(blob)
        predictions = self.net.forward()
        # step 3 - unwrap the predictions to get the object detections 
        class_ids = []
        confidences = []
        boxes = []

        output_data = predictions[0]

        image_width, image_height, _ = input_image.shape
        x_factor = image_width / 640
        y_factor =  image_height / 640
        # print("output data: ", len(output_data))
        for r in range(25200):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:
                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = alex.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])
        if show or save:
            for i in range(len(result_class_ids)):

                box = result_boxes[i]
                class_id = result_class_ids[i]

                cv2.rectangle(image, box, (0, 255, 255), 2)
                cv2.rectangle(image, (box[0], box[1] - 20), (box[0] + box[2], box[1]), (0, 255, 255), -1)
                cv2.putText(image, class_list[class_id], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
        if save:
            cv2.imwrite("test/"+str(alex)+".png", image)
        if show:
            cv2.imshow("output", image)
            cv2.waitKey()
        t2 = time.time()-t1
        print("time: ", t2)
        return result_class_ids, result_confidences, result_boxes, t2

if __name__ == '__main__':
    try:
        node = ObjectDetector()
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
