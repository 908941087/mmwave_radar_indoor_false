#!/usr/bin/env python

'''
This file merge lidar and radar map for building the final map. The final map contains
all obstacles include transparent door.

The final map is just for visualization.
'''

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from classifier import PubThread
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import threading
import numpy as np
import shapely
from shapely.geometry import Polygon, MultiPoint


class MarkerGenerator:
    def __init__(self):
        self.res_marker = Marker()
        self.bbox_id = 0
        self.res_marker.header.frame_id = "/map"
        self.res_marker.header.stamp = rospy.Time.now()

    # Generate 3d bounding box
    def generate_obstacle_bbox(self, polygon, height):
        self.res_marker.type = Marker.LINE_LIST
        self.res_marker.action = Marker.ADD

        self.res_marker.id = self.bbox_id
        self.res_marker.ns = "env_boundary" + str(self.bbox_id)
        self.bbox_id += 1

        # Visualization config
        self.res_marker.scale.x = 0.05
        self.res_marker.color.r = 1.0
        self.res_marker.color.g = 0.0
        self.res_marker.color.b = 0.0
        self.res_marker.color.a = 1.0

        # Data config
        for line_p in polygon:
            t_p1, t_p2 = Point(), Point()
            t_p1.x, t_p1.y, t_p1.z = line_p[0], line_p[1], 0.0
            t_p2.x, t_p2.y, t_p2.z = line_p[0], line_p[1], height
            self.res_marker.points.append(t_p1)
            self.res_marker.points.append(t_p2)
        self.res_marker.points.extend(self.generate_obstacle_ring(polygon, 0.0))
        self.res_marker.points.extend(self.generate_obstacle_ring(polygon, height))

        return self.res_marker

    # Util func for generate bounding box
    @staticmethod
    def generate_obstacle_ring(polygon, height=0.0):
        res_points = []
        for i in range(len(polygon)):
            t_p = Point()
            t_p.x, t_p.y, t_p.z = polygon[i][0], polygon[i][1], height
            res_points.append(t_p)
            if i != 0:
                res_points.append(t_p)
        p_start = Point()
        p_start.x, p_start.y, p_start.z = polygon[0][0], polygon[0][1], height
        res_points.append(p_start)
        return res_points


class EnvMarkers:
    def __init__(self):
        self.door_markers = None
        self.trans_door_markers = None
        self.obstacles = None


if __name__ == '__main__':
    polygon = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (2.0, 1.0), (1.0, 0.0)]
    bbox_pub = rospy.Publisher("/bbox", Marker, queue_size=1)
    bbox_pub_event = threading.Event()
    bbox_pub_thread = PubThread("PubBBox", bbox_pub, bbox_pub_event)

    try:
        rospy.init_node('bbox_test', anonymous=True)
        mark_gen = MarkerGenerator()
        bbox_pub_thread.start()
        bbox_pub_event.clear()
        bbox_pub_thread.data = mark_gen.generate_obstacle_bbox(polygon, 4.0)
        bbox_pub_event.set()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
