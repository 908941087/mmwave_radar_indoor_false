import numpy as np
from scipy.spatial import ConvexHull
from sympy import Point, Polygon

class Cluster(object):
    
    def __init__(self, id, points):
        self.id = id
        self.points = points
        self.hull = None
        self.info = {}

    def getCenter(self):
        return [float(np.average([p[0] for p in self.points])), float(np.average([p[1] for p in self.points]))]

    def getPoints(self):
        return self.points

    def getDensity(self):
        pass

    def getConvexHull(self):
        if self.hull is None:
            points = np.array(self.points).reshape(-1, 2)
            self.hull = ConvexHull(points)
        return self.hull

    def getArea(self):
        

    def show(self, plt):
        plt.scatter([p[0] for p in self.points], [p[1] for p in self.points], c='r', s=1)
