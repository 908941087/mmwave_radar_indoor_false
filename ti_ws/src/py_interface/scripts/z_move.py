#! /usr/bin/env python

import sys
import rospy, time, tf
import math as math
from geometry_msgs.msg import Twist, Point
from probabilistic_lib.functions import angle_wrap
from threading import Timer
import Queue
import time

def main():
    rospy.init_node('z_move')

    RATE = 10
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)  
    rate = rospy.Rate(RATE)   # 10hz
    flag = 0

    while not rospy.is_shutdown():
        command = Twist()
        command.linear.x = 0.5
        ticks = 5 * RATE;
        for i in range(0, ticks):
            pub.publish(command)
            rate.sleep()
        command.linear.x = 0
        pub.publish(command)
        if flag == 0:
            command.angular.z = angle_wrap(135)
            for i in range(0, 10):
                pub.publish(command)
                rate.sleep()
            flag = 1
        else:
            command.angular.z = angle_wrap(-135)
            for i in range(0, 10):
                pub.publish(command)
                rate.sleep()
            flag = 0
    
if __name__ == '__main__':
    main()
