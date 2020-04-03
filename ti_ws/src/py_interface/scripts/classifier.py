#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from EnvClassifier import points_generator
import threading

from visualization_msgs.msg import Marker


class ClassifierPublisher:
    def __init__(self):
        self.pc2_pub_thread = None
        self.marker_pub_thread = None

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

        pc2_generator = points_generator.PointsGenerator()
        point_cloud2 = sensor_msgs.point_cloud2.read_points(data)
        points = [[i[0], i[1]] for i in point_cloud2]
        classified_points = pc2_generator.generate(points)
        if classified_points is None:
            rospy.loginfo("No enough points for classification")
            return
        res_points = []
        for p in classified_points:
            res_points.append((p[0], p[1], 0.0))
        pub_points = sensor_msgs.point_cloud2.create_cloud(data.header, data.fields, res_points)
        pc2_pub.publish(pub_points)

    @staticmethod
    def pc2_pub_func(data):
        pc2_pub.publish(data)

    @staticmethod
    def marker_pub_func(data):
        marker_pub.publish(data)

if __name__ == '__main__':
    map_classifier = ClassifierPublisher()
    pc2_pub = rospy.Publisher('/classified_point_cloud', PointCloud2, queue_size=10)
    marker_pub = rospy.Publisher("/class_marker", Marker, queue_size=10)
    try:
        map_classifier.classify()
    except rospy.ROSInterruptException:
        pass
