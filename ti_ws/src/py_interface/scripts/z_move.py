#! /usr/bin/env python

import sys
import rospy, tf
from geometry_msgs.msg import Twist
from threading import Timer
from math import radians

def main():
    rospy.init_node('z_move')

    RATE = 10
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)  
    rate = rospy.Rate(RATE)   # 10hz
    flag = 0
    count = 0

    while not rospy.is_shutdown() and count < 10:
        command = Twist()
        command.linear.x = 0.2
        ticks = 5 * RATE
        for i in range(0, ticks):
            pub.publish(command)
            rate.sleep()
        command.linear.x = 0
        if flag == 0:
            command.angular.z = radians(135)
            for i in range(0, RATE):
                pub.publish(command)
                rate.sleep()
            flag = 1
        else:
            command.angular.z = radians(-135)
            for i in range(0, RATE):
                pub.publish(command)
                rate.sleep()
            flag = 0
        count += 1
    
if __name__ == '__main__':
    main()
