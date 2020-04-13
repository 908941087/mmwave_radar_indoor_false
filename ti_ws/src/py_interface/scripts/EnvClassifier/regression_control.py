# coding=utf-8

from linear_regressor import SegmentedLinearRegressor
from points_divider import *
from utils import filter_points, get_rotation_matrix, get_intersection, rotate_line
import pypcd
import numpy as np


class RegressionController:

    def __init__(self, path=None, verbose=False):
        """
        Initialization
        @param path: pcd file path
        @param verbose: if True, print more information in the console
        """
        if path is None:
            self.points = []
        else:
            self.points = [[i['x'], i['y']] for i in pypcd.PointCloud.from_path(path).pc_data]
        self.verbose = verbose
        self.parameters = []
        self.parts = []
        self.intersections = []

    def clear_state(self):
        self.regressors = []
        self.parts = []
        self.intersections = []

    def get_points(self):
        return self.points

    def set_points(self, points):
        self.points = points

    def fit(self, regressor):
        """
        Use linear regressor to fit the points
        @param regressor: instance of regressor
        @return: list of intersection list
        """
        count = 1
        if len(self.parts) == 0: raise Exception("Nothing to fit, check if set_parts() method is called.")
        for part in self.parts:
            if part.isolated:
                self.intersections.append(None)
                self.parameters.append(None)
                continue
            if self.verbose:
                print("\n\n************************ Fitting part {} ************************".format(count))
            self.intersections.append(regressor.process(part.points))
            self.parameters.append(regressor.get_parameters())
            count += 1

    def set_parts(self, divider, resolution=1):
        """
        Divide all points into parts to fit
        @param divider: PointsDivider object, divide points into parts
        @return:
        """
        divider.set_points(self.points)
        divider.set_blocks(resolution)
        self.parts = divider.divide()

    def get_intersections(self):
        return self.intersections


class SimpleRegressionController(RegressionController):
    # Manually marked region to fit

    def set_parts(self):
        limits = [[-2, -1, float('-inf'), float('inf')], \
                  [4, 5, float('-inf'), float('inf')], \
                  [float('-inf'), float('inf'), -1, -0.2], \
                  [float('-inf'), float('inf'), 3, 4]]
        count = 0
        for limit in limits:
            points = filter_points(self.points, limit[0], limit[1], limit[2], limit[3])
            self.parts.append(ContinuousPart(points))
            count += 1

    def get_intersections(self):
        parameters = [p[0] for p in self.parameters]
        print(parameters)
        points = []
        points.append(get_intersection(parameters[0], parameters[2]))
        points.append(get_intersection(parameters[0], parameters[3]))
        points.append(get_intersection(parameters[1], parameters[2]))
        points.append(get_intersection(parameters[1], parameters[3]))
        intersections = []
        intersections.append([[points[0][0], points[0][1] + 2], points[1]])
        intersections.append([points[0], points[2]])
        intersections.append([points[2], points[3]])
        intersections.append([points[3], points[1]])
        return intersections
