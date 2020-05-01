import numpy as np
from sympy import Point, Polygon
from sympy.geometry import convex_hull

class Cluster(object):
    
    def __init__(self, id, points):
        self.id = id
        self.points = []
        for p in points:
            self.points.append(Point(p[0], p[1]))
        self.convex_hull = None
        self.area = None

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
        return [float(np.average([p[0] for p in self.points])), float(np.average([p[1] for p in self.points]))]

    def getPoints(self):
        return self.points

    def getDensity(self):
        return len(self.points) / float(self.getArea())

    def getConvexHull(self):
        if self.convex_hull is None:
            self.convex_hull = convex_hull(self.points)
        return self.convex_hull

    def getArea(self):
        if self.area is None:
            self.area = self.getConvexHull().area()
        return self.area

    def getInfo(self):
        return {"Id": self.id, "Points_count": len(self.points), "Area": self.getArea()}

    def show(self, plt):
        plt.scatter([p[0] for p in self.points], [p[1] for p in self.points], c='r', s=1)
