import PointCloudFilter
import Frame
import sensor_msgs.point_cloud2
import open3d as o3d
import rospy
import point_cloud
import numpy as np
from math import sqrt


class PCL_process:

    def __init__(self):
        self.pre_frame = None
        self.cur_frame = None
        self.pc2 = None

    def process(self, pc2):
        self.pc2 = pc2
        self.passthrough_filter()
        self.stablize_preframe()
        self.statistical_outlier_removal()

    def passthrough_filter(self):
        # rospy.loginfo("Passthrough ====================")

        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        for p in points:
            if sqrt(p[0] * p[0] + p[1] * p[1]) < 5.0 and p[3] > 200.0:
            # if p[3] > 200.0:
                res_points.append((p[0], p[1], p[2], p[3]))
                rospy.loginfo("SNR: %s", str(p[3]))
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def stablize_preframe(self):
        # rospy.loginfo("Stablize ====================")
        self.pre_frame = self.cur_frame
        self.cur_frame = Frame.Frame(self.pc2)
        if self.pre_frame is not None:
            PointCloudFilter.find_neighbors(self.pre_frame, self.cur_frame)
            self.pc2 = self.pre_frame.generate()

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
