#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from srv import *

class nav_test():
    def __init__(self):
        rospy.init_node('localisation_node', anonymous=True)
        self.server = rospy.Service("nav", nav, self.doNav)
        self.rate = rospy.Rate(10)
        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=1)

    def doNav(self,request):
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(request.speed)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(np.clip(request.steering,-23,23))+'}'
        if request.speed < 0.001:
            msg.data = '{"action":"3","steerAngle":'+str(0)+'}'
            self.cmd_vel_pub.publish(msg)
            return 'stop'
        t_end = rospy.Time.now() + rospy.Duration(request.time)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_pub.publish(msg2)
            self.rate.sleep()
        msg.data = '{"action":"3","steerAngle":'+str(0)+'}'
        self.cmd_vel_pub.publish(msg)
        return 'stop'

if __name__ == '__main__':
    try:
        node = nav_test()
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass