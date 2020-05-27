#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs.point_cloud2
from EnvClassifier import GroupingTraker
import threading
from datetime import datetime

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Polygon


class PubBlock:
    def __init__(self, thread_handle, event_handle, sub_func, callback_func=None):
        self.thread_handle = thread_handle
        self.event_handle = event_handle
        self.sub_func = sub_func
        self.callback_func = callback_func
        self.data = None

    def acquire_data(self):
        self.data = self.sub_func()
        if self.data is None:
            return False
        return True

    def update_data(self):
        self.event_handle.clear()
        if self.callback_func is not None:
            self.data = self.callback_func(self.data)
        self.thread_handle.data = self.data
        self.event_handle.set()
        rospy.loginfo("Pub " + self.thread_handle.thread_name)


class SubThread(threading.Thread):
    def __init__(self, thread_name="SubThread", duration=10.0):
        super(SubThread, self).__init__(name=thread_name)
        self.thread_name = thread_name
        self.duration = duration
        self._pub_blocks = {}

    def register_thread_CB(self, pub_block):
        self._pub_blocks[pub_block.thread_handle.thread_name] = pub_block

    def run(self):
        rospy.loginfo("Start sub thread: " + self.thread_name)
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('/filtered_point_cloud_centers', PointCloud2, timeout=None)
            laser_grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=None)
            point_cloud2 = sensor_msgs.point_cloud2.read_points(data)
            points = [[i[0], i[1]] for i in point_cloud2]
            # Generate points and marks
            if points is not None and len(points) > 0:
                env = self.group_tracker.getEnv(points, laser_grid)
                classified_points = env.getPoints()
                classified_marks = env.generateInfoMarkers()
            else:
                rospy.sleep(self.duration)
                continue
            # self.group_tracker.showClusters()

            # Transform classified points to rosmsg
            if classified_points is None or classified_marks is None:
                rospy.loginfo("No enough points for classification")
                rospy.sleep(2)
                continue
            # Just store generated points TODO: dilation
            res_points = []
            for p in classified_points:
                res_points.append((p.x, p.y, 0.0))
            pub_points = sensor_msgs.point_cloud2.create_cloud(data.header, data.fields, res_points)

            # Publish points and markers(result of classification)
            self.pc2_pub_event.clear()
            self.pc2_pub_thread.data = pub_points
            self.pc2_pub_event.set()
            rospy.loginfo("Change point cloud")

            # Modify old markers
            self.marker_array_pub_event.clear()
            t_markers = self.marker_array_pub_thread.data
            if t_markers is not None:
                for t_marker in t_markers:
                    t_marker.action = Marker.DELETE
            self.marker_array_pub_thread.data = t_markers
            self.marker_array_pub_event.set()
            # rospy.sleep(0.5)

            self.marker_array_pub_event.clear()
            self.marker_array_pub_thread.data = classified_marks
            self.marker_array_pub_event.set()
            rospy.loginfo("Change Markers")
            # Sleep every interval
            rospy.sleep(self.duration)


class Distributer:
    def __init__(self, publisher, call_back):
        self.publisher = publisher
        self.call_back = call_back

    def deal_and_pub(self, data):
        res = self.call_back(data)
        if isinstance(res, MarkerArray):
            res_delete = []
            for mark in res:
                mark.action = Marker.DELETE
            self.publisher.publish(res_delete)
        self.publisher.publish(res)

class PubThread(threading.Thread):
    def __init__(self, thread_name, pub_event, pub_rate=3):
        super(PubThread, self).__init__(name=thread_name)
        self.thread_name = thread_name
        self.data = None
        self.pub_event = pub_event
        self.pub_rate = pub_rate
        self.distributers = []

    def register_distributer(self, distributer):
        self.distributers.append(distributer)

    def run(self):
        rospy.loginfo("Start pub thread: " + self.thread_name)
        while not rospy.is_shutdown():
            while self.pub_event.is_set():
                if self.data is not None:
                    # rospy.loginfo("Pub " + self.thread_name)
                    for distributer in self.distributers:
                        distributer.deal_and_pub(self.data)
                    rospy.sleep(0.2)
                else:
                    rospy.loginfo("Nothing to Pub " + self.thread_name)


def pc2_grid_sub_func():
    # Get point data
    group_tracker = GroupingTraker.GroupingTracker()
    data = rospy.wait_for_message('/filtered_point_cloud_centers', PointCloud2, timeout=None)
    laser_grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=None)
    point_cloud2 = sensor_msgs.point_cloud2.read_points(data)
    points = [[i[0], i[1], i[2]] for i in point_cloud2]
    # Generate points and marks
    if points is not None and len(points) > 0:
        env = group_tracker.getEnv(points, laser_grid)
    else:
        rospy.loginfo("No enough points for classification")
        return None
    # Generate markers
    return env

def get_marker_array_callback(data):
    return data.generateInfoMarkers()

def get_polygon_array_callback(data):
    return data.generateShapeMarkers()

def get_transparent_obstacle_callback(data):
    return data.generateTransparentObstacleMarkers()

if __name__ == '__main__':
    # Pub components
    marker_array_pub = rospy.Publisher("/class_marker", MarkerArray, queue_size=1)
    polygon_array_pub = rospy.Publisher("/polygon_marker", MarkerArray, queue_size=1)
    transparent_obstacle_pub = rospy.Publisher("/transparent_obstacle", Polygon, queue_size=1)

    marker_distributer = Distributer(marker_array_pub, get_marker_array_callback)
    polygon_distributer = Distributer(polygon_array_pub, get_polygon_array_callback)
    transparent_obstacle_distributer = Distributer(transparent_obstacle_pub, get_transparent_obstacle_callback)

    pub_event = threading.Event()
    pub_thread = PubThread("PubMarkerArray", pub_event)
    pub_thread.register_distributer(marker_distributer)
    pub_thread.register_distributer(polygon_distributer)
    pub_thread.register_distributer(transparent_obstacle_distributer)
    marker_pub_block = PubBlock(pub_thread, pub_event, pc2_grid_sub_func)

    pc2_sub_thread = SubThread("SubPC2", duration=4.0)
    pc2_sub_thread.register_thread_CB(marker_pub_block)
    try:
        rospy.init_node('map_classifier', anonymous=True)
        pc2_sub_thread.start()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
