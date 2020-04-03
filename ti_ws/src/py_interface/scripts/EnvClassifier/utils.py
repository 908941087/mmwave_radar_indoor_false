# coding=utf-8

import pypcd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
import random
from scipy import special


def get_points_from_pcd(path):
    return [[i['x'], i['y']] for i in pypcd.PointCloud.from_path(path).pc_data]


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


def filter_points(points, x_low=float('-inf'), x_high=float('inf'), y_low=float('-inf'), y_high=float('inf')):
    result = [i for i in points if x_low <= i[0] < x_high and y_low <= i[1] < y_high]
    return result


def get_xy_lim(points):
    x = [i[0] for i in points]
    y = [i[1] for i in points]
    return [min(x), max(x), min(y), max(y)]


def get_rectangle(points):
    xy_lim = get_xy_lim(points)
    four_points = [[xy_lim[0], xy_lim[2]], [xy_lim[1], xy_lim[2]], [xy_lim[1], xy_lim[3]], [xy_lim[0], xy_lim[3]]]
    edges = [[four_points[i], four_points[(i + 1) % 4]] for i in range(4)]
    return edges


def get_area(points):
    xy_lim = get_xy_lim(points)
    return abs((xy_lim[1] - xy_lim[0]) * (xy_lim[3] - xy_lim[2]))


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


def get_k_b(points):
    a, b, c = line_fit(points)
    if b != 0:
        return - a / b, - c / b
    elif a != 0:
        return float('inf'), - c / a
    else:
        return float('inf'), float('inf')


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


def line_fit(points):
    """
    并非一元线性回归，详见
    https://blog.csdn.net/liyuanbhu/article/details/50866802?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task
    """
    count = len(points)
    if (count < 2): return 0, 0, 0
    x_mean = sum([p[0] for p in points]) / float(count)
    y_mean = sum([p[1] for p in points]) / float(count)

    Dxx = Dxy = Dyy = 0.0
    for i in range(count):
        Dxx += pow((points[i][0] - x_mean), 2)
        Dxy += (points[i][0] - x_mean) * (points[i][1] - y_mean)
        Dyy += pow((points[i][1] - y_mean), 2)

    Lambda = ((Dxx + Dyy) - np.sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0
    den = np.sqrt(Dxy * Dxy + (Lambda - Dxx) * (Lambda - Dxx))

    a = Dxy / den
    b = (Lambda - Dxx) / den
    c = - a * x_mean - b * y_mean
    return a, b, c


def get_gaussian_weight(n):
    weights = [0 for i in range(n)]
    sum = float(1 << n)
    for i in range((n + 1) / 2):
        weight = special.binom(n, i) / sum
        weights[i] = weight
        weights[n - 1 - i] = weight
    return weights


if __name__ == "__main__":
    points = get_points_from_pcd("ti_ws/src/py_interface/scripts/EnvClassifier/south_one.pcd")
    points = filter_points(points, 0, 2, 0, 2)
    print(get_center(points))
