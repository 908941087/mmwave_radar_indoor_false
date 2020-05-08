import numpy as np
import math
from shapely.geometry import LineString, Point, box, MultiPoint
from centerline.geometry import Centerline
from ..Entity import Wall

def lineFit(points):
    count = len(points)
    if (count < 2): return 0, 0, 0
    x_mean = sum([p.x for p in points]) / float(count)
    y_mean = sum([p.y for p in points]) / float(count)

    Dxx = Dxy = Dyy = 0.0
    for i in range(count):
        Dxx += pow((points[i].x - x_mean), 2)
        Dxy += (points[i].x - x_mean) * (points[i].y - y_mean)
        Dyy += pow((points[i].y - y_mean), 2)

    Lambda = ((Dxx + Dyy) - np.sqrt(float((Dxx - Dyy) ** 2 + 4 * Dxy * Dxy))) / 2.0
    den = np.sqrt(float(Dxy * Dxy + (Lambda - Dxx) * (Lambda - Dxx)))

    a = Dxy / den
    b = (Lambda - Dxx) / den
    c = - a * x_mean - b * y_mean
    
    temp = points.convex_hull.buffer(1)
    bounds = temp.bounds
    k = - a / b
    b = - c / b
    p1 = Point(bounds[0], k * bounds[0] + b)
    p2 = Point(bounds[2], k * bounds[2] + b)
    return LineString([p1, p2])


def circleFit(points):
    pass


def wallFit(cluster):
    WALL_VARIANCE_THRESHOLD = 0.05
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
            bounds = points.bounds
            b = box(bounds[0], bounds[1], bounds[2], bounds[3])
            segments.append(b.intersection(line))
        else:
            parts = branch(points)
            for part in parts:
                if isinstance(part, MultiPoint) and len(part) > LEAST_POINTS_COUNT_TO_FIND_WALL: wallFitCore(part)

    def branch(points):
        bounds = points.bounds
        x_mid = (bounds[0] + bounds[2]) / 2.0
        y_mid = (bounds[1] + bounds[3]) / 2.0
        result = []
        result.append(points.intersection(box(bounds[0], bounds[1], x_mid, y_mid)))
        result.append(points.intersection(box(x_mid, bounds[1], bounds[2], y_mid)))
        result.append(points.intersection(box(bounds[0], y_mid, x_mid, bounds[3])))
        result.append(points.intersection(box(x_mid, y_mid, bounds[2], bounds[3])))
        return result

    def assessWallFit(points, line):
        distances = []
        for point in points:
            distances.append(line.distance(point))
        var = np.var(distances)
        return var < WALL_VARIANCE_THRESHOLD, sum(distances)

    wallFitCore(cluster.getPoints())
    return Wall(cluster.getId(), cluster.getConcaveHull(), segments, 2 * total_dists[0] / float(cluster.getPointsCount()))


# TODO: test
def boneFit(cluster):
    centerline = Centerline(cluster.getConcaveHull())
    total_dists = centerline.geoms
    return Wall(cluster.getId(), cluster.getConcaveHull(), centerline.geoms, 4)