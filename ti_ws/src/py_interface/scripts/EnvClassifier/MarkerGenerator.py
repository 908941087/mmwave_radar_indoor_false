#!/usr/bin/env python

'''
This file merge lidar and radar map for building the final map. The final map contains
all obstacles include transparent door.

The final map is just for visualization.
'''

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MarkerGenerator:
    def __init__(self):
        pass

    def generate_entity_info_marker(self, info):
        t_marker = Marker()
        t_marker.header.frame_id = "/map"
        t_marker.header.stamp = rospy.Time.now()
        t_marker.ns = "info"

        t_marker.id = info["Id"]
        # Type
        t_marker.type = Marker.TEXT_VIEW_FACING
        loc = info["Location"]
        t_marker.text = "Name: " + info["Name"] + "\nx: " + str(loc[0]) + "\ny: " + str(loc[1])
        # Size
        t_marker.scale.x = 0.2
        t_marker.scale.y = 0.2
        t_marker.scale.z = 0.2

        # ADD/DELETE
        t_marker.action = Marker.ADD

        # Pose
        t_marker.pose.position.x = loc[0]
        t_marker.pose.position.y = loc[1]
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

        t_marker.lifetime = rospy.Duration.from_sec(6.0)
        return t_marker

    # Generate a 3d bounding box for one polygon
    def generate_obstacle_bbox(self, marker_id, polygon, height):
        res_marker = Marker()
        res_marker.header.frame_id = "/map"
        res_marker.header.stamp = rospy.Time.now()
        res_marker.type = Marker.LINE_LIST
        res_marker.action = Marker.MODIFY

        res_marker.id = marker_id
        res_marker.ns = "obstacles"

        # Visualization config
        res_marker.scale.x = 0.05
        res_marker.color.r = 1.0
        res_marker.color.g = 0.0
        res_marker.color.b = 0.0
        res_marker.color.a = 1.0

        # Data config
        for line_p in polygon:
            t_p1, t_p2 = Point(), Point()
            t_p1.x, t_p1.y, t_p1.z = line_p[0], line_p[1], 0.0
            t_p2.x, t_p2.y, t_p2.z = line_p[0], line_p[1], height
            res_marker.points.append(t_p1)
            res_marker.points.append(t_p2)
        res_marker.points.extend(self.generate_obstacle_ring(polygon, 0.0))
        res_marker.points.extend(self.generate_obstacle_ring(polygon, height))

        res_marker.lifetime = rospy.Duration.from_sec(6.0)
        return res_marker

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
