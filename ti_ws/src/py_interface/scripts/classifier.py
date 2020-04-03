#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from EnvClassifier import GroupingTrakerModule
import threading

from visualization_msgs.msg import MarkerArray


class ClassifierPublisher:
    def __init__(self):
        self.pc2_pub_thread = None
        self.marker_pub_thread = None
        self.group_tracker = None

    def classify(self):
        rospy.init_node('map_classifier', anonymous=True)
        # Log
        tip_str = "Publish classified map " + str(rospy.get_time())
        rospy.loginfo(tip_str)
        # pc2_data = rospy.wait_for_message('/filtered_point_cloud_centers', PointCloud2, timeout=None)
        # if pc2_data is not None:
        #     classifier_cb(pc2_data)
        rospy.Subscriber('/filtered_point_cloud_centers', PointCloud2, self.classifier_cb)
        # Sleep every interval
        # rospy.sleep(pub_interval)
        rospy.spin()

    @staticmethod
    def classifier_cb(data):
        rate = rospy.Rate(20)
        point_cloud2 = sensor_msgs.point_cloud2.read_points(data)
        points = [[i[0], i[1]] for i in point_cloud2]
        # Generate points and marks
        classified_points = map_classifier.group_tracker.generate_points_per_cluster(points)
        classified_marks = map_classifier.group_tracker.generate_makers()

        if classified_points is None:
            rospy.loginfo("No enough points for classification")
            return
        # Just store generated points TODO: dilation
        res_points = []
        for i_cluster in classified_points:
            for p in i_cluster:
                res_points.append((p[0], p[1], 0.0))
        pub_points = sensor_msgs.point_cloud2.create_cloud(data.header, data.fields, res_points)
        # Publish points and markers(result of classification)
        pc2_pub.publish(pub_points)
        marker_array_pub.publish(classified_marks)

    # @staticmethod
    # def pc2_pub_func(data):
    #     pc2_pub.publish(data)
    #
    # @staticmethod
    # def marker_pub_func(data):
    #     marker_pub.publish(data)


if __name__ == '__main__':
    map_classifier = ClassifierPublisher()
    map_classifier.group_tracker = GroupingTrakerModule.GroupingTracker()
    pc2_pub = rospy.Publisher('/classified_point_cloud', PointCloud2, queue_size=10)
    marker_array_pub = rospy.Publisher("/class_marker", MarkerArray, queue_size=10)
    try:
        map_classifier.classify()
    except rospy.ROSInterruptException:
        pass
