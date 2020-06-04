#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
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
        if self.callback_func is not None:
            self.data = self.callback_func(self.data)
        self.event_handle.clear()
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
            for pub_block in self._pub_blocks.values():
                if not pub_block.thread_handle.isAlive():
                    pub_block.thread_handle.start()
                if pub_block.acquire_data():
                    pub_block.update_data()
                else:
                    continue
            # Sleep every interval
            rospy.sleep(self.duration)


class Distributer:
    def __init__(self, publisher, call_back):
        self.publisher = publisher
        self.call_back = call_back

    def deal_and_pub(self, data):
        res = self.call_back(data)
        if isinstance(res, MarkerArray):
            for mark in res:
                mark.action = Marker.DELETE
            self.publisher.publish(res)
            for mark in res:
                mark.action = Marker.ADD
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
                if rospy.is_shutdown():
                    break
                if self.data is not None:
                    # rospy.loginfo("Pub " + self.thread_name)
                    for distributer in self.distributers:
                        distributer.deal_and_pub(self.data)
                    rospy.sleep(2.0)
                else:
                    rospy.loginfo("Nothing to Pub " + self.thread_name)


def pc2_grid_sub_func():
    # Get point data
    group_tracker = GroupingTraker.GroupingTracker()
    data = rospy.wait_for_message('/filtered_point_cloud_centers', PointCloud2, timeout=None)
    laser_grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=None)
    robot_pose = rospy.wait_for_message('/odom', Odometry, timeout=None)
    point_cloud2 = sensor_msgs.point_cloud2.read_points(data)
    points = [[i[0], i[1], i[2]] for i in point_cloud2]
    # Generate points and marks
    if points is not None and len(points) > 0:
        env = group_tracker.getEnv(points, laser_grid, robot_pose)
    else:
        rospy.loginfo("No enough points for classification")
        return None
    # Generate markers
    return env

def get_marker_array_callback(data):
    rospy.loginfo("Updating obstacle infos.")
    return data.generateInfoMarkers()

def get_polygon_array_callback(data):
    rospy.loginfo("Updating obstacle polygons.")
    return data.generateShapeMarkers()

def get_transparent_obstacle_callback(data):
    rospy.loginfo("Updating transparent obstacles.")
    return data.generateTransparentObstacleMarkers()

if __name__ == '__main__':
    # Pub components
    marker_array_pub = rospy.Publisher("/class_marker", MarkerArray, queue_size=1)
    polygon_array_pub = rospy.Publisher("/polygon_marker", MarkerArray, queue_size=100)
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

    pc2_sub_thread = SubThread("SubPC2", duration=8.0)
    pc2_sub_thread.register_thread_CB(marker_pub_block)
    try:
        rospy.init_node('map_classifier', anonymous=True)
        pc2_sub_thread.start()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
