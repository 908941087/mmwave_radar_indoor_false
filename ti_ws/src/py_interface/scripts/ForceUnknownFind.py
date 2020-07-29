#!/usr/bin/env python
#
# Created by zgh on 2020/4/15.
#

# ROS imports
import rospy
import tf
import math

#matshow
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

# Numpy
import numpy as np

current_position_ = np.zeros(2)
current_orientation_ = 0.0
desired_position_ = np.zeros(2)

# Map data
dat = None
wid = None
heigh = None
res = None
xorg = None
yorg = None
# Map for DFS
filtered_map = None
RevMap = None

"""
Using dfs to find the first unknown place to reach.
input: "force_unknown_find"  occupancy_grid(map)  
output: start_point, goal_point
"""


def visual_pub(tx, ty):
    target_pose = PoseStamped()

    # Transform res_point to goal_point
    target_pose.header.seq = 0
    target_pose.header.stamp = rospy.get_rostime()
    target_pose.header.frame_id = "map"
    target_pose.pose.position.z = 0.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.w = 1.0

    target_pose.pose.position.x = xorg + tx * res
    target_pose.pose.position.y = yorg + ty * res
    goal_pub.publish(target_pose)


def log_neighbor(point_index, neighbor_field = 4):
    global filtered_map
    m_wid = filtered_map.shape[0]
    m_heigh = filtered_map.shape[1]

    #rospy.loginfo("Neibor of point: %d, %d", point_index[0], point_index[1])
    res_point = point_index
    p_point = np.zeros(2, int)
    for x_d in range(-neighbor_field, neighbor_field + 1):
        p_point[0] = x_d + res_point[0]
        if p_point[0] < 0 or p_point[0] >= m_wid: continue
        for y_d in range(-neighbor_field, neighbor_field + 1):
            p_point[1] = y_d + res_point[1]
            if p_point[1] < 0 or p_point[1] >= m_heigh: continue
            rospy.loginfo("%d, %d, %d", p_point[0], p_point[1], filtered_map[p_point[0], p_point[1]])

def ObstacleInflation(filtered_map, neighbor_field = 5):
    RevMap = np.zeros(filtered_map.shape, dtype=np.int)
    # Format map
    for i in range(filtered_map.shape[0]):
        for j in range(filtered_map.shape[1]):
            if filtered_map[i, j] == 100:
                RevMap[i, j] = 1
    for i in range(filtered_map.shape[0]):
        for j in range(filtered_map.shape[1]):
            if RevMap[i, j] == 1:
                t_point = np.zeros(2, int)
                for x_d in range(-neighbor_field, neighbor_field + 1):
                    t_point[0] = x_d + i
                    if t_point[0] < 0 or t_point[0] >= filtered_map.shape[0]: continue
                    for y_d in range(-neighbor_field, neighbor_field + 1):
                        t_point[1] = y_d + j
                        if t_point[1] < 0 or t_point[1] >= filtered_map.shape[1]: continue
                        filtered_map[t_point[0]][t_point[1]] = int(100)
    return filtered_map                  


def ForceUnknownFindCB(msg):
    global dat, wid, heigh, res, filtered_map, current_position_, xorg, yorg
    rospy.loginfo("Calculate goal in nearest unknown part.")
    # If no map input, return
    if dat is None:
        return

    m_xorg = xorg
    m_yorg = yorg
    m_wid = wid
    m_heigh = heigh
    m_res = res

    filtered_map = np.array(dat).reshape((m_wid, m_heigh))
    rospy.loginfo("Map: %s, %s, %d, %d, %d, %d, %s", m_xorg, m_yorg, m_wid, m_heigh, filtered_map.shape[0],
                  filtered_map.shape[1], m_res)

    RevMap = np.zeros(filtered_map.shape, dtype=np.int)
    # Format map
    for i in range(m_wid):
        for j in range(m_heigh):
            if filtered_map[i, j] == -1:
                filtered_map[i, j] = 255
    # show the map
    # plt.matshow(filtered_map)
    # plt.show()

    #inflation the obstacle of filtered_map
    filtered_map =  ObstacleInflation(filtered_map)


    point_index = np.zeros(2, dtype=np.int)
    point_index[1] = int((current_position_[0] - m_xorg) / res)
    point_index[0] = int((current_position_[1] - m_yorg) / res)
    rospy.loginfo("location:  %d, %d", point_index[0], point_index[1])
    #clear the start point
    t_point = np.zeros(2, int)
    robot_size = 4
    # for i in range(-robot_size, robot_size + 1):
    #     t_point[0] = i + point_index[0]
    #     for j in range(-robot_size, robot_size + 1):
    #         t_point[1] = j + point_index[1]
    #         filtered_map[t_point[0]][t_point[1]] = int(0)
    #log_neighbor(point_index)
    target_pose = PoseStamped()

    # Transform res_point to goal_point
    target_pose.header.seq = 0
    target_pose.header.stamp = rospy.get_rostime()
    target_pose.header.frame_id = "map"
    target_pose.pose.position.z = 0.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.w = 1.0

    res_point = FindUnkownAreaBFS(point_index, RevMap.copy(), m_xorg, m_yorg)
    if res_point is not None:
        if res_point[0] != point_index[0] or res_point[1] != point_index[1]:
            target_pose.pose.position.x = m_xorg + res_point[1] * res
            target_pose.pose.position.y = m_yorg + res_point[0] * res
            rospy.loginfo("Publish neareast unknown point value" + str(
                filtered_map[int(res_point[0])][int(res_point[1])]))
            goal_pub.publish(target_pose)
            p_point = np.zeros(2, int)
            #log_neighbor(res_point)
            # for x_d in range(-1, 2):
            #     p_point[0] = x_d + res_point[0]
            #     if p_point[0] < 0 or p_point[0] >= m_wid: continue
            #     for y_d in range(-1, 2):
            #         p_point[1] = y_d + res_point[1]
            #         if p_point[1] < 0 or p_point[1] >= m_heigh: continue
            #         rospy.loginfo("%d, %d, %d", p_point[0], p_point[1], filtered_map[p_point[0], p_point[1]])
        else:
            target_pose.pose.position.x = m_xorg + (3+res_point[0]) * res
            target_pose.pose.position.y = m_yorg + (3+res_point[1]) * res
            goal_pub.publish(target_pose)
    else:
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.0
        goal_pub.publish(target_pose)
        rospy.loginfo("Returning!")
        rospy.on_shutdown(myhook)


# DFS version
def FindUnkownArea(point_index, local_map):
    # visual_pub(point_index[0], point_index[1])

    global filtered_map
    m_wid = filtered_map.shape[0]
    m_heigh = filtered_map.shape[1]
    if int(local_map[int(point_index[0])][int(point_index[1])]) == 1:
        return None
    else:
        local_map[int(point_index[0])][int(point_index[1])] = 1

    if int(filtered_map[int(point_index[0])][int(point_index[1])]) == 100:
        return None
    elif int(filtered_map[int(point_index[0])][int(point_index[1])]) == 255:
        if judge_neighbor(point_index):
            return point_index

    t_point = np.zeros(2)
    local_map[int(point_index[0])][int(point_index[1])] = 1
    for x_d in range(-1, 2)[::-1]:
        t_point[0] = x_d + point_index[0]
        if t_point[0] < 0 or t_point[0] >= m_wid: continue
        for y_d in range(-1, 2)[::-1]:
            t_point[1] = y_d + point_index[1]
            if t_point[1] < 0 or t_point[1] >= m_heigh: continue
            tmp_res = FindUnkownArea(t_point, local_map.copy())
            if tmp_res is not None:
                return tmp_res
    return None



def judge_oldgoal(cur_point, m_xorg, m_yorg):
    global filtered_map, oldgoalmap, res, old_xorg, old_yorg
    checkflag = True
    if oldgoalmap is None:
        rospy.loginfo("judge the goal!")
        oldgoalmap = np.zeros(filtered_map.shape, dtype=np.int)
        oldgoalmap[int(cur_point[0])][int(cur_point[1])] = 1
        old_xorg = m_xorg
        old_yorg = m_yorg
        return checkflag
    else:
        t_oldgoalmap = np.zeros(filtered_map.shape, dtype=np.int)
        for i in range(0, oldgoalmap.shape[0]):
            for j in range(0, oldgoalmap.shape[1]):
                if oldgoalmap[i][j] == 1:
                    x_old = old_xorg + j * res
                    y_old = old_yorg + i * res
                    i_now = int((y_old - m_yorg) / res)
                    j_now = int((x_old - m_xorg) / res)
                    t_oldgoalmap[i_now][j_now] = oldgoalmap[i][j]
                    #rospy.loginfo("oldmap_shape[0]: %s; oldmapshape[1]: %s; x_old: %s; y_old: %s;filtermap_shape[0]: %s; filtermap_shape[1]: %s; i_now: %s; j_now: %s; ", oldgoalmap.shape[0], oldgoalmap.shape[1], i, j,filtered_map.shape[0], filtered_map.shape[1], i_now, j_now)
    oldgoalmap = t_oldgoalmap
    old_xorg = m_xorg
    old_yorg = m_yorg
    rospy.loginfo("judge the goal2!")
    if oldgoalmap[int(cur_point[0])][int(cur_point[1])] == 1:
        checkflag = False
        return checkflag
    else:
        oldgoalmap[int(cur_point[0])][int(cur_point[1])] = 1
        return checkflag


def judge_neighbor(point_index, neighbor_field = 6):
    global filtered_map
    m_wid = filtered_map.shape[0]
    m_heigh = filtered_map.shape[1]
    m_allcount = 0
    # m_allcount = (neighbor_field * 2 + 1) * (neighbor_field * 2 + 1)
    m_count = 0
    # m_ratio = 0.0

    all_unknown_flag = True
    delta = neighbor_field
    t_point = np.zeros(2, int)
    for x_d in range(-delta, delta + 1):
        t_point[0] = x_d + point_index[0]
        if t_point[0] < 0 or t_point[0] >= m_wid: continue
        for y_d in range(-delta, delta + 1):
            t_point[1] = y_d + point_index[1]
            if t_point[1] < 0 or t_point[1] >= m_heigh: continue
            m_count += 1
            if filtered_map[int(t_point[0])][int(t_point[1])] == 255:
                m_allcount += 1
    m_ratio = float(m_allcount) / m_count
    #rospy.loginfo("ratio, %s, %d, %d", m_ratio, m_count, m_allcount)
    return m_ratio

#ensure the goal is legal
def judge_obstacle_neighbor(point_index, neighbor_field = 6):
    global filtered_map
    m_wid = filtered_map.shape[0]
    m_heigh = filtered_map.shape[1]

    all_unknown_flag = True
    delta = neighbor_field
    t_point = np.zeros(2, int)
    for x_d in range(-delta, delta + 1):
        t_point[0] = x_d + point_index[0]
        if t_point[0] < 0 or t_point[0] >= m_wid: continue
        for y_d in range(-delta, delta + 1):
            t_point[1] = y_d + point_index[1]
            if t_point[1] < 0 or t_point[1] >= m_heigh: continue
            if filtered_map[int(t_point[0])][int(t_point[1])] == 100:
                all_unknown_flag = False
                rospy.loginfo("this goal near the obstacle! ignore this choice!")
                break
    return all_unknown_flag


# BFS version
def FindUnkownAreaBFS(point_index, local_map, m_xorg, m_yorg):
    global filtered_map
    m_wid = filtered_map.shape[0]
    m_heigh = filtered_map.shape[1]

    p_stack = [point_index]
    local_map[point_index[0]][point_index[1]] = int(1)

    while len(p_stack) > 0:
        #rospy.loginfo("stack size: %d", len(p_stack))
        if len(p_stack) == 1:
            cur_point = p_stack.pop(0)
            log_neighbor(cur_point)
            rospy.loginfo("p_stack size = 1")
            p_stack.append(cur_point.copy())
        cur_point = p_stack.pop(0)        
        if judge_neighbor(cur_point) > 0.8 and (cur_point[0] != point_index[0] and cur_point[1] != point_index[1]) and filtered_map[cur_point[0]][cur_point[1]] == 255:
            if judge_oldgoal(cur_point, m_xorg, m_yorg):
                return cur_point
            else:
                for x_d in range(-8, 9):
                    t_point[0] = x_d + cur_point[0]
                    if t_point[0] < 0 or t_point[0] >= m_wid: continue
                    for y_d in range(-8, 9):
                        t_point[1] = y_d + cur_point[1]
                        if t_point[1] < 0 or t_point[1] >= m_heigh: continue
                        local_map[t_point[0]][t_point[1]] = int(1)
                # num = 36
                # while len(p_stack) > 0 and num > 0:
                #     p_stack.pop(0)
                #     num = num -1

        t_point = np.zeros(2, int)
        for x_d in range(-1, 2):
            t_point[0] = x_d + cur_point[0]
            if t_point[0] < 0 or t_point[0] >= m_wid: continue
            for y_d in range(-1, 2):
                t_point[1] = y_d + cur_point[1]
                if t_point[1] < 0 or t_point[1] >= m_heigh: continue
                if int(local_map[t_point[0]][t_point[1]]) == 1: continue
                if filtered_map[t_point[0]][t_point[1]] == 100: continue
                if x_d == 0 and y_d == 0: continue
                local_map[t_point[0]][t_point[1]] = int(1)
                p_stack.append(t_point.copy())
    return None


"""
Return the angle normalized between [-pi, pi].
"""


def odomCallback(odometry_msg):
    global current_position_
    global current_orientation_
    current_position_[0] = odometry_msg.pose.pose.position.x
    current_position_[1] = odometry_msg.pose.pose.position.y
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y,
         odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w])
    current_orientation_ = wrapAngle(y)
    return


def OccupancyGridCallback(msg):
    global dat, wid, heigh, res, xorg, yorg
    dat = msg.data
    heigh = msg.info.width
    wid = msg.info.height
    res = msg.info.resolution
    xorg = msg.info.origin.position.x
    yorg = msg.info.origin.position.y


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


def myhook():
    print "shutdown force_unknown_find_handler!"


def test():
    global filtered_map
    print("Start test")
    test_map = np.array([100, 100, 100, 255,
                         255, 100, 0, 255,
                         255, 100, 0, 255,
                         255, 100, 100, 255]).reshape((4, 4))
    revMap = np.zeros(test_map.shape, int)
    point_index = np.zeros(2, int)
    point_index[0], point_index[1] = 2, 2
    filtered_map = test_map
    res = FindUnkownArea(point_index, revMap)
    print res
    print("Test success")


if __name__ == '__main__':
    global oldgoalmap 
    oldgoalmap = None
    rospy.init_node('force_unknown_find_handler', log_level=rospy.INFO)
    rospy.loginfo("Start find unknown point")
    force_unknown_find_sub = rospy.Subscriber("/force_find", Int8, ForceUnknownFindCB, queue_size=1)
    odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback, queue_size=1)
    map_sub_ = rospy.Subscriber("/map", OccupancyGrid, OccupancyGridCallback, queue_size=1)
    goal_pub = rospy.Publisher("/unknown_goal", PoseStamped, queue_size=1)
    test()
    rospy.spin()
