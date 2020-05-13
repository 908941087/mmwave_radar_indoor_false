# coding=utf-8

import pypcd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
import random
from scipy import special
from scipy.spatial import ConvexHull
from PCBasics import *

"""
TODO:
1. implement geometry use sympy
"""


def get_intersection(line1, line2):
    # y = ax + b, line = [a, b]
    if line1[0] == line2[0]:
        raise Exception("Line 1 and line 2 are parallel, NO intersection can be found.")
    else:
        intersection = []
        intersection.append((line2[1] - line1[1]) / float((line1[0] - line2[0])))  # (b2 - b1) / (a1 - a2)
        intersection.append(intersection[0] * line1[0] + line1[1])  # (b2 - b1) / (a1 - a2) * a1 + b1
        return intersection


def get_avg_line(line1, line2):
    # get the average line from line1 and line2
    slope1 = k2slope(line1[0])
    slope2 = k2slope(line2[0])
    slope = (slope1 + slope2) / 2.0
    if abs(slope1 - slope2) > 90:
        if slope < 90: slope += 90
        else: slope -= 90
    k = slope2k(slope) 
    intersection = get_intersection(line1, line2)
    b = intersection[1] - k * intersection[0]
    return [k, b]


def get_center(points):
    return [float(np.average([p[0] for p in points])), float(np.average([p[1] for p in points]))]


def get_density(points):
    return len(points) / get_area(points)


def diff_vertically(point, line):
    return abs(point[1] - line[0] * point[0] - line[1])


def variance(points, lines, segements):
    result = 0.0
    for segment, line in zip(segements, lines):
        points_seg = [i for i in points if segment[0] <= i[0] < segment[1]]
        for point in points_seg:
            result += pow(diff_vertically(point, line), 2)
    return result


def get_rectangle(points):
    xy_lim = getXYLim(points)
    four_points = [[xy_lim[0], xy_lim[2]], [xy_lim[1], xy_lim[2]], [xy_lim[1], xy_lim[3]], [xy_lim[0], xy_lim[3]]]
    edges = [[four_points[i], four_points[(i + 1) % 4]] for i in range(4)]
    return edges


MIN_RECTANGLE_AREA = 0.07 # square meters

def get_area(cluster):
    return get_area_core(cluster, get_rectangle_area(cluster))


def get_area_core(cluster, rectangle_area):
    if len(cluster) == 0: return 0
    if rectangle_area < MIN_RECTANGLE_AREA: return rectangle_area
    sub_clusters = branch(cluster)
    sub_rectangle_area = rectangle_area / 4.0
    return sum([get_area_core(c, sub_rectangle_area) for c in sub_clusters])


def get_rectangle_area(cluster):
    xy_lim = getXYLim(cluster)
    return (xy_lim[1] - xy_lim[0]) * (xy_lim[3] - xy_lim[2])


def branch(cluster):
        xy_lim = getXYLim(cluster)
        x_mid = (xy_lim[0] + xy_lim[1]) / 2.0
        y_mid = (xy_lim[2] + xy_lim[3]) / 2.0
        # plt.plot([x_mid, x_mid], [xy_lim[2], xy_lim[3]], c='r', linewidth=1, linestyle='dotted')
        # plt.plot([xy_lim[0], xy_lim[1]], [y_mid, y_mid], c='r', linewidth=1, linestyle='dotted')
        result = []
        result.append(filterWithRect(cluster, xy_lim[0], x_mid, xy_lim[2], y_mid))
        result.append(filterWithRect(cluster, x_mid, xy_lim[1], xy_lim[2], y_mid))
        result.append(filterWithRect(cluster, xy_lim[0], x_mid, y_mid, xy_lim[3]))
        result.append(filterWithRect(cluster, x_mid, xy_lim[1], y_mid, xy_lim[3]))
        return result


def get_rotation_matrix(deg):
    # counter-clockwise rotation matrix, center is (0, 0)
    theta = np.radians(deg)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return R


def rotate_line(line, deg):
    """
    @param line: [k, b]
    @param deg: degrees to rotate, counter clockwise
    """
    theta = np.deg2rad(deg)
    coef_a = line[0] * np.cos(theta) + np.sin(theta)
    coef_b = np.cos(theta) - line[0] * np.sin(theta)
    k = coef_a / coef_b
    b = line[1] / coef_b
    return [k, b]


def translate_points(points, new_origin):
    for p in points:
        p[0] -= new_origin[0]
        p[1] -= new_origin[1]
    return points


def scale_points(points, ratio):
    for p in points:
        p[0] *= ratio
        p[1] *= ratio
    return points


def rotate_points(points, angle):
    """
    @param points: points to rotate
    @param angle: degree, angle to rotate, counter clockwise, rotation origin is (0, 0)
    @return points after rotation
    """
    R = get_rotation_matrix(angle)
    new_points = []
    for p in points:
        vec = np.array([p[0], p[1]])
        new_points.append(list(R.dot(vec)))
    return new_points


def get_slope(points):
    """
    Calculate the overall slope for points, 0 ~ 180
    """
    k, b = get_k_b(points)
    return k2slope(k)


def k2slope(k):
    deg = np.rad2deg(np.arctan(k))
    if deg < 0: deg += 180
    return deg


def slope2k(slope):
    if slope == 90: return float('inf')
    angle = np.radians(slope)
    return np.sin(angle) / np.cos(angle)


def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def dist_point2line(p, line):
    a = line[0]
    b = -1
    c = line[1]
    return (a * p[0] + b * p[1] + c) / np.sqrt(a ** 2 + b ** 2)


def safe_divide(a, b):
    while b < 1:
        a *= 10
        b *= 10
    return a / b


def get_gaussian_weight(n):
    weights = [0 for i in range(n)]
    sum = float(1 << n)
    for i in range((n + 1) / 2):
        weight = special.binom(n, i) / sum
        weights[i] = weight
        weights[n - 1 - i] = weight
    return weights


def get_convex_hull(points):
    points = np.array(points).reshape(-1, 2)
    hull = ConvexHull(points)
    return [[points[simplex, 0], points[simplex, 1]] for simplex in hull.simplices]


if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    points = getPCFromPCD("ti_ws/src/py_interface/scripts/EnvClassifier/pcds/south_one.pcd")
    points = filterWithRect(points, 0, 2, 0, 2)
    points = np.array(points).reshape(-1, 2)
    hull = ConvexHull(points)
    plt.plot(points[:,0], points[:,1], 'o')
    plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)

    plt.show()