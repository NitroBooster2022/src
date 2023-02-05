#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    """
    :param data: sensor_msg array containing the image in the Gazsbo format
    :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
    """
    cv_image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("Frame preview", cv_image)
    key = cv2.waitKey(1)
    
            
if __name__ == '__main__':
    global bridge
    bridge = CvBridge()
    global cv_image
    cv_image = np.zeros((640, 480))
    rospy.init_node('CAMnod', anonymous=True)
    image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()