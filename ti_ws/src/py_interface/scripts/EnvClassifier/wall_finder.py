# coding=utf-8

from utils import *
from linear_regressor import LinearRegressor
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
from collections import deque

class WallFinder(object):
    WIDTH_THRESHOLD = 0.32
    LEAST_POINTS_COUNT_TO_FIND_WALL = 20

    def __init__(self):
        self.walls = []
        self.regressor = LinearRegressor()
        self.index = 0
        self.blocks = []

    def fit_a_wall(self, cluster):
        ends = self.regressor.process(cluster)
        line = self.regressor.get_parameters()[0]
        avg_dist = self.get_avg_distance(cluster, line)
        info = {"line": line, "ends":ends, "width": avg_dist * 2}
        return info

    def find_walls_core(self, cluster):
        if cluster is None or len(cluster) == 0:
            return None
        info = self.fit_a_wall(cluster)
        if info["width"] < self.WIDTH_THRESHOLD:
            self.blocks.append(cluster)
            info["ID"] = self.index
            self.walls.append(info)
            self.index += 1
        else:
            clusters = self.branch(cluster)
            for one_cluster in clusters:
                if len(one_cluster) > self.LEAST_POINTS_COUNT_TO_FIND_WALL: self.find_walls_core(one_cluster)
        return self.walls

    def find_walls(self, cluster):
        self.walls = []
        self.index = 0
        self.blocks = []
        self.walls = self.find_walls_core(cluster)
        return self.blocks, self.walls

    @staticmethod
    def branch(cluster):
        xy_lim = get_xy_lim(cluster)
        x_mid = (xy_lim[0] + xy_lim[1]) / 2.0
        y_mid = (xy_lim[2] + xy_lim[3]) / 2.0
        # plt.plot([x_mid, x_mid], [xy_lim[2], xy_lim[3]], c='r', linewidth=1, linestyle='dotted')
        # plt.plot([xy_lim[0], xy_lim[1]], [y_mid, y_mid], c='r', linewidth=1, linestyle='dotted')
        result = []
        result.append(filter_points(cluster, xy_lim[0], x_mid, xy_lim[2], y_mid))
        result.append(filter_points(cluster, x_mid, xy_lim[1], xy_lim[2], y_mid))
        result.append(filter_points(cluster, xy_lim[0], x_mid, y_mid, xy_lim[3]))
        result.append(filter_points(cluster, x_mid, xy_lim[1], y_mid, xy_lim[3]))
        return result

    @staticmethod
    def get_avg_distance(points, line):
        a = line[0]
        b = -1
        c = line[1]
        temp = sum([abs(a * p[0] + b * p[1] + c) for p in points])
        return temp / np.sqrt(a ** 2 + 1) / float(len(points))

    @staticmethod
    def prolong_ends(ends, ratio=1.1):
        """
        Prolong the ends according to the ratio
        """
        mid = [sum([p[0] for p in ends]) / 2.0, sum([p[1] for p in ends]) / 2.0]
        return [[mid[0] - (mid[0] - p[0]) * ratio, mid[1] - (mid[1] - p[1]) * ratio] for p in ends]
