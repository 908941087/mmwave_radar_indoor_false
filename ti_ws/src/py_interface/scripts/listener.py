#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from Filters import Frame
from Filters import PCL_process
# import PointCloudFilter
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

pub = None
robot_loc_pub = None

frame_service = Frame.FrameService()
stablizer = frame_service.get_multi_frame_stablizer(width=8, height=4, resolution=0.5, frame_num=3, threshold=5000)

def callback(data):
    global frame_service, stablizer
    PCL_Ins = PCL_process.PCL_process()
    PCL_Ins.process(frame_service, stablizer, data)
    pub.publish(PCL_Ins.genrate_res())


def show_robot_loc_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    t_marker = Marker()
    t_marker.header.frame_id = "/map"
    t_marker.header.stamp = rospy.Time.now()
    t_marker.ns = "robot_loc"

    t_marker.id = 0

    # Type
    t_marker.type = Marker.TEXT_VIEW_FACING
    t_marker.text = "RobotLoc: " + "\n" + "x: " + str(round(x, 3)) + "\n" + "y: " + str(
        round(y, 3))
    # Size
    t_marker.scale.x = 0.3
    t_marker.scale.y = 0.3
    t_marker.scale.z = 0.3

    # ADD/DELETE
    t_marker.action = Marker.MODIFY

    # Pose
    t_marker.pose.position.x = 5
    t_marker.pose.position.y = 0
    t_marker.pose.position.z = 0.2
    t_marker.pose.orientation.x = 0.0
    t_marker.pose.orientation.y = 0.0
    t_marker.pose.orientation.z = 0.0
    t_marker.pose.orientation.w = 1.0

    # Color
    t_marker.color.r = 0.0
    t_marker.color.g = 1.0
    t_marker.color.b = 0.5
    t_marker.color.a = 1.0

    t_marker.lifetime = rospy.Duration(0.1)

    robot_loc_pub.publish(t_marker)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('mmWaveDataHdl/RScan', PointCloud2, callback)

    rospy.Subscriber('/odom', Odometry, show_robot_loc_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    pub = rospy.Publisher('/py_test', PointCloud2, queue_size=10)
    robot_loc_pub = rospy.Publisher('/robot_loc', Marker, queue_size=10)
    listener()
