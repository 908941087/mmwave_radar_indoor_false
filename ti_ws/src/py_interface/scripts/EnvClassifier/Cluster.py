import numpy as np
from shapely.geometry import LineString, Point, box
from PointCloudOperator import PCBasics
from alphashape import alphashape

class Cluster(object):
    
    def __init__(self, eid, points):
        self.id = eid
        self.points = points
        self.concave_hull = None
        self.concave_hull_points = None

    def distance(self, other):
        return self.points.distance(other)

    def getId(self):
        return self.id

    def getCenter(self):
        return self.points.centroid

    def getPoints(self):
        return self.points

    def getDensity(self):
        return len(self.points) / float(self.getArea())

    def getConcaveHull(self):
        if self.concave_hull is None:
            self.concave_hull = alphashape(self.getPoints(), 2)
        return self.concave_hull

    def getArea(self):
        return abs(self.getConcaveHull().area)

    def getPointsCount(self):
        return len(self.points)

    def getBounds(self):
        return self.points.bounds  # (minx, miny, maxx, maxy)

    def getSegment(self, line):
        bounds = self.getBounds()
        abox = box(bounds[0], bounds[1], bounds[2], bounds[3])
        return abox.intersection(line)

    def getInfo(self):
        return {"Id": self.id, "Points_count": len(self.points), "Area": self.getArea()}

    def show(self, plt, c='r'):
        plt.scatter([p.x for p in self.points], [p.y for p in self.points], c=c, s=1)
