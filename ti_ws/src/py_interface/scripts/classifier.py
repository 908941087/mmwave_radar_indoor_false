#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from EnvClassifier import GroupingTrakerModule
import threading

from visualization_msgs.msg import MarkerArray, Marker


class SubThread(threading.Thread):
    def __init__(self, thread_name, pc2_pub_thread, pc2_pub_event, marker_array_pub_thread, marker_array_pub_event):
        super(SubThread, self).__init__(name=thread_name)
        self.data = None
        self.thread_name = thread_name
        self.pc2_pub_thread = pc2_pub_thread
        self.pc2_pub_event = pc2_pub_event
        self.marker_array_pub_thread = marker_array_pub_thread
        self.marker_array_pub_event = marker_array_pub_event
        self.group_tracker = GroupingTrakerModule.GroupingTracker()
        self.duration = 2.0

    def run(self):
        rospy.loginfo("Start sub thread: " + self.thread_name)
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('/filtered_point_cloud_centers', PointCloud2, timeout=None)
            point_cloud2 = sensor_msgs.point_cloud2.read_points(data)
            points = [[i[0], i[1]] for i in point_cloud2]
            # Generate points and marks
            if points is not None and len(points) > 0:
                classified_points = self.group_tracker.generate_points_per_cluster(points)
                classified_marks = self.group_tracker.generate_makers()
            else:
                rospy.sleep(self.duration)
                continue
            # self.group_tracker.show_clusters()

            # Transform classified points to rosmsg
            if classified_points is None or classified_marks is None:
                rospy.loginfo("No enough points for classification")
                rospy.sleep(2)
                continue
            # Just store generated points TODO: dilation
            res_points = []
            for p in classified_points:
                res_points.append((p[0], p[1], 0.0))
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


class PubThread(threading.Thread):
    def __init__(self, thread_name, publisher, pub_event, pub_rate=3):
        super(PubThread, self).__init__(name=thread_name)
        self.thread_name = thread_name
        self.data = None
        self.publisher = publisher
        self.pub_event = pub_event
        self.pub_rate = pub_rate

    def run(self):
        rospy.loginfo("Start pub thread: " + self.thread_name)
        while not rospy.is_shutdown():
            while self.pub_event.is_set():
                if self.data is not None:
                    # rospy.loginfo("Pub " + self.thread_name)
                    self.publisher.publish(self.data)
                else:
                    rospy.loginfo("Nothing to Pub " + self.thread_name)
            rospy.sleep(2.0)


if __name__ == '__main__':
    # Pub components
    pc2_pub = rospy.Publisher('/classified_point_cloud', PointCloud2, queue_size=1)
    marker_array_pub = rospy.Publisher("/class_marker", MarkerArray, queue_size=1)
    pc2_pub_event = threading.Event()
    marker_array_pub_event = threading.Event()

    pc2_pub_thread = PubThread("PubPC2", pc2_pub, pc2_pub_event)
    marker_array_pub_thread = PubThread("PubMarkerArray", marker_array_pub, marker_array_pub_event)
    pc2_sub_thread = SubThread("SubPC2", pc2_pub_thread, pc2_pub_event, marker_array_pub_thread, marker_array_pub_event)

    try:
        rospy.init_node('map_classifier', anonymous=True)
        pc2_sub_thread.start()
        pc2_pub_thread.start()
        marker_array_pub_thread.start()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
