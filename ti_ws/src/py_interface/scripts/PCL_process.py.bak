import Frame
import sensor_msgs.point_cloud2
import open3d as o3d
import rospy
import point_cloud
import numpy as np
from math import sqrt


class PCL_process:

    def __init__(self):
        self.pc2 = None

    def process(self, frame_service, stablizer, pc2):
        self.frame_service = frame_service
        self.stablizer = stablizer
        self.pc2 = pc2
        self.passthrough_filter()
        # self.stablize_preframe()
        # self.statistical_outlier_removal()

    def passthrough_filter(self):
        rospy.loginfo("Passthrough ====================")

        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        intensity_list = [p[3] for p in points]
        # ave_intensity = sum(intensity_list) / len(intensity_list)
        # rospy.loginfo("AVE SNR: %s", str(ave_intensity))

        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        for p in points:
            if p[3] > 100 and sqrt(p[0] * p[0] + p[1] * p[1]) < 2.0:
                res_points.append((p[0], p[1], p[2], p[3]))

        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def stablize_preframe(self):
        # rospy.loginfo("Stablize ====================")
        current_frame = self.frame_service.point_cloud_to_frame(self.pc2)
        stable_frame = self.stablizer.update(current_frame)
        if stable_frame is not None:
            self.pc2 = self.frame_service.frame_to_point_cloud(stable_frame)

    def statistical_outlier_removal(self):
        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        for p in points:
            res_points.append([p[0], p[1], p[2]])
        cloud = np.array(res_points, dtype='float32', order='C')
        cloud = point_cloud.RemoveStatisticalOutliers(cloud, 5, 0.05)

        res_points = []
        for i in range(cloud.shape[0]):
            p = cloud[i]
            res_points.append((p[0], p[1], p[2], 40.0))
            print("SNR:   " + str(p))
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def genrate_res(self):
        return self.pc2
    # def filter_points(self, pc2):
