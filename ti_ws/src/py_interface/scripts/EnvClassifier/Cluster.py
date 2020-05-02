import numpy as np
from sympy.geometry import convex_hull, Line, Segment, Point, Polygon
from sympy.abc import x, y
from PointCloudOperator import PCBasics

class Cluster(object):
    
    def __init__(self, id, points):
        self.id = id
        self.points = []
        for p in points:
            self.points.append(Point(p[0], p[1]))
        self.convex_hull = None
        self.area = None
        self.xy_lim = None

    def distance(self, other):
        if isinstance(other, Cluster):
            center1 = self.getCenter()
            center2 = other.getCenter()
            min_dist = float('inf')
            p2 = None
            for p in other.getPoints():
                temp = center1.distance(p)
                if min_dist > temp:
                    min_dist = temp
                    p2 = p
            min_dist = float('inf')
            for p in self.getPoints():
                temp = p2.distance(p)
                if min_dist > temp:
                    min_dist = temp
            return min_dist
        elif isinstance(other, Point):
            min_dist = float('inf')
            for p in self.getPoints():
                temp = p2.distance(other)
                if min_dist > temp:
                    min_dist = temp
            return min_dist

    def getId(self):
        return self.id

    def getCenter(self):
        return self.getConvexHull().centroid

    def getPoints(self):
        return self.points

    def getDensity(self):
        return len(self.points) / float(self.getArea())

    def getConvexHull(self):
        if self.convex_hull is None:
            self.convex_hull = convex_hull(*self.points)
        return self.convex_hull

    def getArea(self):
        if self.area is None:
            self.area = self.getConvexHull().area
        return self.area

    def getPointsCount(self):
        return len(self.points)

    def getXYLim(self):
        if self.xy_lim is None:
            self.xy_lim = PCBasics.getXYLim(self.getPoints())
        return self.xy_lim

    def getSegment(self, line):
        if not isinstance(line, Line):
            raise TypeError("line must be type of Line")
        xy_lim = self.getXYLim()
        xs = [xy_lim[0], xy_lim[1], line.intersection(Line(Point(0, xy_lim[2]), slope=0))[0][0], line.intersection(Line(Point(0, xy_lim[3]), slope=0))[0][0]]
        xs.sort()
        return Segment(line.intersection(Line(Point(xs[1], 0), Point(xs[1], 1)))[0], line.intersection(Line(Point(xs[2], 0), Point(xs[2], 1)))[0])

    def getInfo(self):
        return {"Id": self.id, "Points_count": len(self.points), "Area": self.getArea()}

    def show(self, plt, c='r'):
        plt.scatter([p[0] for p in self.points], [p[1] for p in self.points], c=c, s=1)
