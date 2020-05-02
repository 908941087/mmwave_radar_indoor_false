import numpy as np
import math
from sympy.abc import x, y
from sympy.geometry import Segment, Line, Point
from PCBasics import getXYLim, filterWithRect
from Entity import Wall

def lineFit(points):
    count = len(points)
    if (count < 2): return 0, 0, 0
    x_mean = sum([p[0] for p in points]) / float(count)
    y_mean = sum([p[1] for p in points]) / float(count)

    Dxx = Dxy = Dyy = 0.0
    for i in range(count):
        Dxx += pow((points[i][0] - x_mean), 2)
        Dxy += (points[i][0] - x_mean) * (points[i][1] - y_mean)
        Dyy += pow((points[i][1] - y_mean), 2)

    Lambda = ((Dxx + Dyy) - np.sqrt(float((Dxx - Dyy) ** 2 + 4 * Dxy * Dxy))) / 2.0
    den = np.sqrt(float(Dxy * Dxy + (Lambda - Dxx) * (Lambda - Dxx)))

    a = Dxy / den
    b = (Lambda - Dxx) / den
    c = - a * x_mean - b * y_mean
    return Line(a*x + b*y + c)


def circleFit(points):
    pass


def wallFit(cluster):
    WALL_VARIANCE_THRESHOLD = 10
    LEAST_POINTS_COUNT_TO_FIND_WALL = 20
    segments = []
    total_dists = [0] # use list instead of int to let the nested function modify it

    def wallFitCore(points):
        if points is None or len(points) == 0:
            return None
        line = lineFit(points)
        is_valid_fit, dists = assessWallFit(points, line)
        if is_valid_fit:
            total_dists[0] += dists
            segments.append(cluster.getSegment(line))
        else:
            parts = branch(points)
            for part in parts:
                if len(part) > LEAST_POINTS_COUNT_TO_FIND_WALL: wallFitCore(part)

    def branch(points):
        xy_lim = getXYLim(points)
        x_mid = (xy_lim[0] + xy_lim[1]) / 2.0
        y_mid = (xy_lim[2] + xy_lim[3]) / 2.0
        result = []
        result.append(filterWithRect(points, xy_lim[0], x_mid, xy_lim[2], y_mid))
        result.append(filterWithRect(points, x_mid, xy_lim[1], xy_lim[2], y_mid))
        result.append(filterWithRect(points, xy_lim[0], x_mid, y_mid, xy_lim[3]))
        result.append(filterWithRect(points, x_mid, xy_lim[1], y_mid, xy_lim[3]))
        return result

    def assessWallFit(points, line):
        distances = []
        for point in points:
            distances.append(line.distance(point))
        var = np.var(distances)
        return var < WALL_VARIANCE_THRESHOLD, sum(distances)

    wallFitCore(cluster.getPoints())
    return Wall(cluster.getId(), segments, 2 * total_dists[0] / float(cluster.getPointsCount()))

