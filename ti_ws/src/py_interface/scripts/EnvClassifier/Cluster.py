class Cluster(object):
    
    def __init__(self, id, points):
        self.id = id
        self.points = points
        self.hull = None
        self.info = {}

    def getCenter(self):
        pass

    def getPoints(self):
        return self.points

    def getDensity(self):
        pass

    def getArea(self):
        pass

    def show(self, plt):
        plt.scatter([p[0] for p in self.points], [p[1] for p in self.points], c='r', s=1)
