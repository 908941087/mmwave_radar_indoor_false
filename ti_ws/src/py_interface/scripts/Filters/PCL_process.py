import Frame
import sensor_msgs.point_cloud2
# import open3d as o3d
import rospy
# import point_cloud
import numpy as np
from math import sqrt, floor, pi, sin, cos, tan


class PCL_process:

    def __init__(self):
        self.pc2 = None
        self.delta = 0.10
        self.heat_map = None
        self.min_x = None
        self.max_x = None
        self.min_y = None
        self.max_y = None

        # For testing and tracing
        self.delta_angle = pi * (-5) / 180
        self.enable_trace = False
        self.reflect_trace_cnt = 0.0
        self.min_prob_thre = 5

    def process(self, frame_service, stablizer, pc2):
        self.frame_service = frame_service
        self.stablizer = stablizer
        self.pc2 = pc2
        # self.adjust_perspective()
        self.adjust_spherical()
        # self.handle_reflection()
        self.passthrough_filter()
        # self.stablize_preframe()
        # self.statistical_outlier_removal()
        # self.add_z_info()

    def adjust_spherical(self):
        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        points_list = [(p[0], p[1], p[2], p[3]) for p in points]
        for p in points_list:
            x, y, z, i = p[0], p[1], p[2], p[3]
            # r*cos(elev) = sqrt(x**2+y**w)
            # 1/cos(elev)
            mul_fac = sqrt(x ** 2 + y ** 2 + z ** 2) / sqrt(x ** 2 + y ** 2)
            res_points.append((x * mul_fac, y * mul_fac, z * mul_fac, i))
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def adjust_perspective(self, direct="y"):
        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        points_list = [(p[0], p[1], p[2], p[3]) for p in points]
        ang = self.delta_angle
        for p in points_list:
            x, y, z, i = p[0], p[1], p[2], p[3]
            if direct == "y":
                x_1 = x * cos(ang) - z * sin(ang)
                y_1 = y
                z_1 = x * sin(ang) + z * cos(ang)
            elif direct == "x":
                x_1 = x
                y_1 = y * cos(ang) + z * sin(ang)
                z_1 = -y * sin(ang) + z * cos(ang)
            else:  # direct == "z":
                x_1 = x * cos(ang) + y * sin(ang)
                y_1 = -x * sin(ang) + y * cos(ang)
                z_1 = z
            res_points.append((x_1, y_1, z_1, i))
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def passthrough_filter(self, dim=3):
        if self.enable_trace:
            rospy.loginfo("Passthrough ====================")
        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        points_list = [(p[0], p[1], p[2], p[3]) for p in points]

        # points_list.sort(key=lambda p: p[3])
        FiltOutRate = 0.2
        for p in points_list[int(FiltOutRate * len(points_list)):]:
            t_dis = sqrt(p[0] * p[0] + p[1] * p[1])
            if 6.0 > t_dis > 0.5:
                if 1.0 > p[2] > 0.2 or dim == 2:
                    res_points.append((p[0], p[1], 0.0, p[3]))
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def handle_reflection(self):
        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        points_list = [[p[0], p[1], p[2], p[3]] for p in points]
        if len(points_list) <= 1:
            return
        # Update heatmap
        self.min_x, self.max_x = min([p[0] for p in points_list]), max([p[0] for p in points_list])
        self.min_y, self.max_y = min([p[1] for p in points_list]), max([p[1] for p in points_list])
        self.heat_map = np.zeros((int((self.max_x - self.min_x) / self.delta) + 1,
                                  int((self.max_y - self.min_y) / self.delta) + 1))
        for p in points_list:
            self.heat_map[self.hm_axis(p)[0]][self.hm_axis(p)[1]] += 1.0

        # Reduce reflection
        for p in points_list:
            prob = 0.0
            res_p = None
            dis = sqrt(p[0] ** 2 + p[1] ** 2)
            if dis >= 2.0:  # minimum reflect distance is about 0.75m
                for step in range(1, int(dis / 1.0)):
                    tp = [p[i] / step for i in range(2)]
                    near_prob = self.cal_neighbor_count(tp, step)
                    if near_prob < self.min_prob_thre: near_prob = 0
                    if near_prob * step > prob:
                        prob = near_prob * step
                        res_p = [p[i] / step for i in range(4)]
                if res_p is not None:
                    if res_p[0] != p[0] or res_p[1] != p[1]:
                        self.reflect_trace_cnt += 1
                    p = res_p
            res_points.append((p[0], p[1], p[2], p[3]))
        if self.enable_trace:
            # self.reflect_trace_rate /= len(points_list)
            rospy.loginfo("Reflect cnt : %d", self.reflect_trace_cnt)

        if res_points is None or len(res_points) <= 1:
            return
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def cal_neighbor_count(self, point, step):
        (px, py) = self.hm_axis(point)
        cnt = 0
        for i in range(-step, step):
            if self.heat_map.shape[0] > px + i >= 0:
                for j in range(-step, step):
                    if self.heat_map.shape[1] > py + j >= 0:
                        cnt += self.heat_map[px + i][py + j]
        return cnt

    def hm_axis(self, point):
        return int(floor((point[0] - self.min_x) / self.delta)), int(floor((point[1] - self.min_y) / self.delta))

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
        if len(res_points) <= 1:
            return
        cloud = np.array(res_points, dtype='float32', order='C')
        cloud = point_cloud.RemoveStatisticalOutliers(cloud, 5, 0.05)

        res_points = []
        for i in range(cloud.shape[0]):
            p = cloud[i]
            res_points.append((p[0], p[1], p[2], 40.0))
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def add_z_info(self, height=0.5, step=10):
        points = sensor_msgs.point_cloud2.read_points(self.pc2)
        res_points = []
        points_list = [(p[0], p[1], p[2], p[3]) for p in points]

        for p in points_list:
            for i in range(step):
                res_points.append((p[0], p[1], p[2] + (i * height / step), 40.0))
        self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)

    def genrate_res(self):
        return self.pc2
    # def filter_points(self, pc2):
