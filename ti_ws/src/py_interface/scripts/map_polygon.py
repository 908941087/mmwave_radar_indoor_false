#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS imports
import rospy
import tf
import math

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

# Numpy
import numpy

polygon_data = None

def polygonCallback(msg):
    global polygon_data
    polygon_data = msg.data
    # map = numpy.array(dat).reshape((wid, heigh))
    # polygon_line_map = numpy.zeros(map.shape, dtype=numpy.int)
    #
    # # 坐标转换
    # polygon_len = len(polygon_data)
    # point_index = numpy.zeros(2, dtype=numpy.int)
    #
    # polygon_data_list = []  # 断点在网格中的坐标
    # rospy.loginfo("the number of the polygon points: %s", polygon_len)
    # for i in range(0, polygon_len):
    #     point_index[1] = int((polygon_data[i].x - xorg) / res)
    #     point_index[0] = int((polygon_data[i].y - yorg) / res)
    #     polygon_data_list.append(point_index)
    #
    # # 直线求点
    # polygon_line_data_list = []
    # for i in range(0, polygon_len, 2):
    #     x1 = polygon_data_list[i][0]
    #     y1 = polygon_data_list[i][1]
    #     x2 = polygon_data_list[i + 1][0]
    #     y2 = polygon_data_list[i + 1][1]
    #     if x1 < 0 or x2 < 0 or x1 >= wid or x2 >= wid or y1 < 0 or y2 < 0 or y1 >= heigh or y2 >= heigh:
    #         rospy.loginfo("negative point: %s", i)
    #         continue
    #     else:
    #         point_index[0] = x1
    #         point_index[1] = y1
    #         polygon_line_data_list.append(point_index)
    #         k = (y1 - y2) / (x1 - x2)
    #         if x1 < x2:
    #             step = 1
    #         else:
    #             step = -1
    #         for i in range(x1 + 1, x2, step):
    #             temp_y = int(k * (i - x1) + y1)
    #             point_index[0] = i
    #             point_index[1] = temp_y
    #             polygon_line_data_list.append(point_index)
    #
    # # 发布加入polygon点了以后的新地图
    # polygon_map = OccupancyGrid()
    # polygon_map.header = msg.header
    # polygon_map.header.stamp = rospy.get_rostime()
    # polygon_map.info = msg.info
    # datalist = numpy.zeros(wid * heigh, dtype=numpy.int8)
    # for i in polygon_line_data_list:
    #     map[i[0]][i[1]] = 100
    # for i in range(wid):
    #     for j in range(heigh):
    #         datalist[i * heigh + j] = map[i][j]
    # polygon_map.data = datalist
    # map_polygon_pub.publish(polygon_map)
    # rospy.loginfo("publish map!")



def OccupancyGridCallback(msg):
    global dat, wid, heigh, res, xorg, yorg, polygon_data
    dat = msg.data
    heigh = msg.info.width
    wid = msg.info.height
    res = msg.info.resolution
    xorg = msg.info.origin.position.x
    yorg = msg.info.origin.position.y
    rospy.loginfo("receive map! %s", wid)

    #测试数据
    # polygon_data = []
    # point1 = Point32()
    # point1.x = 0.0
    # point1.y = 1.0
    # point1.z = 0.0
    # rospy.loginfo("point one: %s", point1)
    # polygon_data.append(point1)
    # point2 = Point32()
    # point2.x = 0.0
    # point2.y = 0.0
    # point2.z = 0.0
    # rospy.loginfo("point one: %s", point2)
    # polygon_data.append(point2)
    # for each in polygon_data:
    #     print("%s",each)


    #新地图
    map = numpy.array(dat).reshape((wid, heigh))
    polygon_line_map = numpy.zeros(map.shape, dtype=numpy.int)

    # 坐标转换

    polygon_len = len(polygon_data)
    point_index = numpy.zeros(2, dtype=numpy.int)

    polygon_data_list = []  # 断点在网格中的坐标
    rospy.loginfo("the number of the polygon points: %s", polygon_len)
    for i in range(0, polygon_len):
        rospy.loginfo("point one: %s, %s", polygon_data[i].x, polygon_data[i].y)
        point_index_temp = numpy.zeros(2, dtype=numpy.int)
        point_index_temp[1] = int((polygon_data[i].x - xorg) / res)
        point_index_temp[0] = int((polygon_data[i].y - yorg) / res)
        rospy.loginfo("point one: %s", point_index)
        print(i)
        polygon_data_list.append(point_index_temp)
        print("%s", polygon_data_list)

    # 直线求点
    polygon_line_data_list = []
    for i in range(0, polygon_len, 2):
        x1 = polygon_data_list[i][0]
        y1 = polygon_data_list[i][1]
        print("%s,%s",x1,y1)
        temp = i + 1
        x2 = polygon_data_list[temp][0]
        y2 = polygon_data_list[temp][1]
        print("%s,%s",x2,y2)
        if x1 < 0 or x2 < 0 or x1 >= wid or x2 >= wid or y1 < 0 or y2 < 0 or y1 >= heigh or y2 >= heigh:
            rospy.loginfo("negative point: %s", i)
            continue
        else:
            point_index1 = numpy.zeros(2, dtype=numpy.int)
            point_index1[0] = x1
            point_index1[1] = y1
            # rospy.loginfo("point one: %s, %s", x1, y1)
            polygon_line_data_list.append(point_index1)
            point_index2 = numpy.zeros(2, dtype=numpy.int)
            point_index2[0] = x2
            point_index2[1] = y2
            # rospy.loginfo("point two: %s, %s", x2, y2)
            polygon_line_data_list.append(point_index2)
            k = (y1 - y2) / (x1 - x2)
            if x1 < x2:
                step = 1
            else:
                step = -1
            for i in range(x1 + 1, x2, step):
                point_index3 = numpy.zeros(2, dtype=numpy.int)
                temp_y = int(k * (i - x1) + y1)
                point_index3[0] = i
                point_index3[1] = temp_y
                polygon_line_data_list.append(point_index3)

    # 发布加入polygon点了以后的新地图
    rospy.loginfo("直线上的网格数量：%s", len(polygon_line_data_list))
    polygon_map = OccupancyGrid()
    polygon_map.header = msg.header
    polygon_map.header.stamp = rospy.get_rostime()
    polygon_map.info = msg.info
    datalist = numpy.zeros(wid * heigh, dtype=numpy.int8)
    for i in polygon_line_data_list:
        map[i[0]][i[1]] = 100
    for i in range(wid):
        for j in range(heigh):
            datalist[i * heigh + j] = map[i][j]
    polygon_map.data = datalist
    map_polygon_pub.publish(polygon_map)
    rospy.loginfo("publish map!")





if __name__ == '__main__':
    rospy.init_node('map_polygon_node', log_level=rospy.INFO)
    rospy.loginfo("Start add polygon in map, and publish new map named polygon_map")
    polygon_sub = rospy.Subscriber("/transparent_obstacle", Polygon, polygonCallback, queue_size=1)
    map_sub = rospy.Subscriber("/map", OccupancyGrid, OccupancyGridCallback, queue_size=1)
    map_polygon_pub = rospy.Publisher("/polygon_map", OccupancyGrid, queue_size=1)
    rospy.spin()
