#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('autonomous_explore_map_plan')
import rospy
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from nav_msgs.msg import OccupancyGrid

#Numpy
import numpy as np

current_position_ = np.zeros(2)
current_orientation_ = 0.0
desired_position_ = np.zeros(2)

tel_pub = None
dat = None
wid = None
heigh = None
res = None
xorg = None
yorg = None

def force_move_Callback(msg):
    # print (msg)
    print "enter force_move"
    global current_position_, desired_position_, current_orientation_
    rate = rospy.Rate(10)   # 10hz
    ticks = 20
    safety_dis = 0.3
    command = Twist()
    desired_position_[0] = current_position_[0] + math.cos(current_orientation_) * safety_dis
    desired_position_[1] = current_position_[1] + math.sin(current_orientation_) * safety_dis
    obs1 = check_unknown_obstacle()
    if obs1 == False :
        command.linear.x = 0.1
        for i in range(0, ticks):
            tel_pub.publish(command)
            rate.sleep()
        return
    # rotate 90 
    desired_position_[0] = current_position_[0] - math.sin(current_orientation_) * safety_dis
    desired_position_[1] = current_position_[1] + math.cos(current_orientation_) * safety_dis
    obs2 = check_unknown_obstacle()
    if obs2 == False :
        command.angular.z = wrapAngle(90)
        for i in range(0, 10):
            tel_pub.publish(command)
            rate.sleep()
        command.angular.z = 0
        command.linear.x = 0.1
        for i in range(0, ticks):
            tel_pub.publish(command)
            rate.sleep()
        return
    # rotate 180
    desired_position_[0] = current_position_[0] - math.cos(current_orientation_) * safety_dis
    desired_position_[1] = current_position_[1] - math.sin(current_orientation_) * safety_dis
    obs3 = check_unknown_obstacle()
    if obs3 == False :
        command.angular.z = wrapAngle(180)
        for i in range(0, 10):
            tel_pub.publish(command)
            rate.sleep()
        command.angular.z = 0
        command.linear.x = 0.1
        for i in range(0, ticks):
            tel_pub.publish(command)
            rate.sleep()
        return
    # rotate 270
    desired_position_[0] = current_position_[0] + math.sin(current_orientation_) * safety_dis
    desired_position_[1] = current_position_[1] - math.cos(current_orientation_) * safety_dis
    obs4 = check_unknown_obstacle()
    if obs4 == False :
        command.angular.z = wrapAngle(-90)
        for i in range(0, 10):
            tel_pub.publish(command)
            rate.sleep()
        command.angular.z = 0
        command.linear.x = 0.1
        for i in range(0, ticks):
            tel_pub.publish(command)
            rate.sleep()
        return

def odomCallback(odometry_msg):
    global current_position_
    global current_orientation_
    current_position_[0] = odometry_msg.pose.pose.position.x
    current_position_[1] = odometry_msg.pose.pose.position.y
    (r, p, y) = tf.transformations.euler_from_quaternion([odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w])
    current_orientation_ = wrapAngle(y)
    return

def OccupancyGridCallback(msg):
    global dat, wid, heigh, res, xorg, yorg
    dat = msg.data
    wid = msg.info.width
    heigh = msg.info.height
    res = msg.info.resolution
    xorg = msg.info.origin.position.x
    yorg = msg.info.origin.position.y

def check_unknown_obstacle():
    # 1. defined unit vector from current pos to goal pos (gazebo unit)
    # 2. define segments in gazebo unit to assess obstacle
    # 3. for i = all seg, convert each to cell unit and check value
    global current_position_, desired_position_
    global dat, wid, heigh, res, xorg, yorg

    dist_to_goal_xy = cal_dist_to_goal_xy()
    unit_distance = 0.1

    x_unit_vect = (desired_position_[0] - current_position_[0])*1.1 #/ dist_to_goal_xy
    y_unit_vect = (desired_position_[1] - current_position_[1])*1.1 #/ dist_to_goal_xy
    num_seg = int(round(dist_to_goal_xy / unit_distance))
    obstacle = False

    if (dat!=None and wid!=None and heigh!=None):
        data1 = np.reshape(dat, (wid ,heigh), order="F")
        data2 = np.asarray(data1)
        x_org = xorg
        y_org = yorg
        x = current_position_[0]
        y = current_position_[1]
    else:
        print("empty, return")
        return True
    
    for i in range(1, num_seg+1):
        x_gazebo =  x + x_unit_vect/num_seg * i
        y_gazebo =  y + y_unit_vect/num_seg * i
        dist = math.sqrt(pow(x-x_gazebo,2)+pow(y-y_gazebo,2))
        x_map, y_map = gazebo2map(x_gazebo, y_gazebo, x_org, y_org)

        # window around (x_map, y_map) all will be checked
        win = np.zeros((5, 5)) 
        win = data2[x_map-2:x_map+3, y_map-2:y_map+3]
        if np.sum([win]) >= 100 and dist < 2:
        #if data2[x_map, y_map] == 100: # obstacle
            obstacle = True
            return obstacle
                #break
    return obstacle

def gazebo2map(x_gazebo, y_gazebo, x_org, y_org):
    # convert units from gazebo to map
    global dat, wid, heigh, res, xorg, yorg
    x_map = int(math.floor((x_gazebo - x_org) / res))
    y_map = int(math.floor((y_gazebo - y_org) / res))

    if x_map < 0:
        x_map = 0
    elif x_map > wid - 2:
        x_map = wid -1

    if y_map < 0:
        y_map = 0
    elif y_map > heigh  -1:
        y_map = heigh -1
  
    return x_map, y_map

def wrapAngle(ang):
    """
    Return the angle normalized between [-pi, pi].
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang

def cal_dist_to_goal_xy():
    global current_position_, desired_position_
    return math.sqrt(pow(current_position_[0]-desired_position_[0],2)+pow(current_position_[1]-desired_position_[1],2))
      

if __name__ == '__main__':
    rospy.init_node('force_move_handler', log_level=rospy.INFO)
    odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback, queue_size = 1)
    force_move_sub = rospy.Subscriber("/force_move", Int8, force_move_Callback, queue_size = 1)
    tel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    map_sub_ = rospy.Subscriber("/map", OccupancyGrid, OccupancyGridCallback, queue_size = 1)

    rospy.spin()
    
