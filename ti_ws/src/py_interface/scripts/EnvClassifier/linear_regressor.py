# coding=utf-8

import pypcd
from utils import *
import numpy as np
import sys
from math import floor, ceil
from scipy.optimize import curve_fit

"""
TODO:
""" 

class RegressorBase(object):

    def __init__(self, verbose=False):
        self.parameters = [] # y = k * x + b, [k, b]
        self.verbose = verbose

    def get_parameters(self):
        return self.parameters

    def process(self, points):
        pass


class SegmentedLinearRegressor(RegressorBase):

    def __init__(self, segments_count, verbose=False):
        self.segments_count = segments_count
        self.parameters = [] # y = k * x + b, [k, b]
        self.verbose = verbose

    def process(self, points):
        """
        用拉格朗日乘数法，对点进行分段线性拟合
        @param points: points need to fit
        @return: list of intersections
        """
        self.parameters = []
        # Rotate points to horizontal
        slope = get_slope(points)
        matrix = get_rotation_matrix(-slope)
        new_points = []
        for p in points:
            vec = np.array([p[0],p[1]])
            new_points.append(list(matrix.dot(vec)))
        
        # get interval and steps
        interval = []
        interval.append(min([i[0] for i in new_points]))
        interval.append(max([i[0] for i in new_points]))
        step = (interval[1] - interval[0]) / float(self.segments_count)

        # get points ready
        x = [i[0] for i in new_points]
        y = [i[1] for i in new_points]

        # get dimension 
        param_count = self.segments_count * 2
        dimension = self.segments_count * 3 - 1

        # way to get the matrix
        def getRows(index):
            """
            计算矩阵的每一行，一次计算两行
            """
            row1 = [0 for i in range(dimension)]
            row2 = [0 for i in range(dimension)]
            left = interval[0] + step * index
            right = left + step
            points = [p for p in new_points if left <= p[0] < right]
            row1[index * 2] = sum([i[0] * i[0] for i in points])
            row1[index * 2 + 1] = sum([i[0] for i in points])
            row2[index * 2] = row1[index * 2 + 1]
            row2[index * 2 + 1] = len(points)
            if index != 0: 
                row1[param_count + index - 1] = -0.5
                row2[param_count + index - 1] = 0.5
            if index != param_count / 2 - 1: 
                row1[param_count + index] = 0.5
                row2[param_count + index] = -0.5
            two_elems = [0, 0]
            two_elems[0] = sum([i[0] * i[1] for i in points])
            two_elems[1] = sum([i[1] for i in points])
            return [row1, row2], two_elems

        # solve the equation
        a = [] # Ax = b, a will be used to generate A
        b = []
        for i in range(self.segments_count): # generate the first segments_count * 2 rows
            two_rows,  two_elems = getRows(i) # i is the segment count
            a += two_rows
            b += two_elems
        for i in range(self.segments_count - 1):
            temp = [0 for j in range(dimension)]
            temp[i * 2] = self.interval[0] + step * (i + 1)
            temp[i * 2 + 1] = 1
            temp[i * 2 + 2] = -temp[i * 2]
            temp[i * 2 + 3] = -1
            a.append(temp)
            b.append(0)
        A = np.matrix(a)
        b = np.array(b)
        singular = False
        if np.linalg.cond(A) < 1/sys.float_info.epsilon: # to avoid singular matrix
            result = np.linalg.solve(A, b)
            parameters = [[result[i * 2], result[i * 2 + 1]] for i in range(self.segments_count)]
            for i in range(len(parameters)):
                self.parameters.append(rotate_line(parameters[i], slope))
        else:
            singular = True

        # rotate things back
        interval = [min([i[0] for i in points]), max([i[1] for i in points])]

        # print everything
        if self.verbose:
            np.set_printoptions(precision=3, suppress=True)
            print("\nA: ")
            print(A)
            if not singular:
                print("\nx: ")
                print(result)
            else:
                print("\nNOTICE: A is singular, no result is available.")
            print("\nb: ")
            print(b)
            print("\nSegments: " + str(self.segments_count))
            print("\nk and b: ")
            print([[round(i[0], 3), round(i[1], 3)]for i in self.parameters])
            print("\nInterval: ")
            print(interval)

        # return intersections
        x = []
        y = []
        for i in range(len(self.parameters) + 1):
            if i == 0: # 左边端点
                param = self.parameters[0]
                x.append(interval[0])
                y.append(x[0] * param[0] + param[1])
            elif i == len(self.parameters): # 右边端点
                param = self.parameters[i - 1]
                x.append(interval[1])
                y.append(x[len(x) - 1] * param[0] + param[1])
            else: # 两条折线的交点
                param1 = self.parameters[i - 1]
                param2 = self.parameters[i]
                intersection = get_intersection(param1, param2)
                x.append(intersection[0])
                y.append(intersection[1])
        return [[i, j] for i, j in zip(x, y)]


class LinearRegressor(RegressorBase):
    
    def process(self, points):
        k, b = get_k_b(points)
        self.parameters = [[k, b]]
        xy_lim = get_xy_lim(points)
        if k == 0: 
            return [[xy_lim[0], b], [xy_lim[1], b]]
        elif k == float('inf') and b != float('inf'):
            return [[b, xy_lim[2]], [b, xy_lim[3]]]
        else:
            x = [xy_lim[0], xy_lim[1], (xy_lim[2] - b) / k, (xy_lim[3] - b) / k]
            x.sort()
            return [[i, k * i + b] for i in x if x[0] < i < x[3]]


class GaussianRegressor(RegressorBase):

    def __init__(self, verbose=False):
        self.parameters = [] # y = k * x + b, [k, b]
        self.verbose = verbose
        self.X_points=[]
        self.Y_points=[]
        
    def process(self,points):
        #print("%%%%%%%%%%%%%%%%",len(points))
        self.X_points=[]
        self.Y_points=[]
        for i in range(len(points)):
            self.X_points.append(points[i][0])
            self.Y_points.append(points[i][1])
        #print("self.X_points长度：",len(self.X_points))
        #print(self.X_points)
        #print("points:",points)
        #a,u,sig=get_param() 
        #controller = GaussianController(path='four_walls.pcd', verbose=verbose)
        #parts=controller.get_parts()
        #for part in parts:
        X_array=np.array(self.X_points)
        Y_array=np.array(self.Y_points)
        if (0<get_slope(points) and get_slope(points)<45) or (135<get_slope(points) and get_slope(points)<180): 
            popt, pcov = curve_fit(self.gaussian_mix, X_array,Y_array,maxfev=500000)
            return [popt[0],popt[1],popt[2]]
        else:
            popt, pcov = curve_fit(self.gaussian_mix, Y_array, X_array,maxfev=500000)
            return [popt[0],popt[1],popt[2]]

    def gaussian_mix(self,x,a,u,sig):
        return a * np.exp(-((x - u) / sig) ** 2)  

    def get_X_points(self):
        return self.X_points       


if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from utils import get_points_from_pcd, get_xy_lim, get_slope

    def abline(slope, intercept):
        """Plot a line from slope and intercept"""
        axes = plt.gca()
        x_vals = np.array(axes.get_xlim())
        y_vals = intercept + slope * x_vals
        plt.plot(x_vals, y_vals, '--')

    points = get_points_from_pcd("four_walls.pcd")

    padding = 0.2
    xy_lim = get_xy_lim(points)
    ratio = (xy_lim[3] - xy_lim[2]) / float(xy_lim[1] - xy_lim[0])
    fig = plt.figure(figsize=(10 * ratio, 10))
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.grid(True, linewidth=0.5, color='#666666', linestyle='dotted')
    ax1.axis([xy_lim[0] - padding, xy_lim[1] + padding, xy_lim[2] - padding, xy_lim[3] + padding])

    points = filter_points(points, 4, 5, 2, 3)
    # points = [[4, 2.0], [4.003, 2.3], [4.05, 2.6], [4.1, 2.9]]
    reg = LinearRegressor(True)
    intersections = reg.process(points)
    ax1.scatter([p[0] for p in points], [p[1] for p in points], color='red', s=4)

    param = reg.get_parameters()[0]
    print(param)
    # abline(param[0], param[1])

    ax1.plot([i[0] for i in intersections], [i[1] for i in intersections], color='blue', linewidth=2)

    plt.show()