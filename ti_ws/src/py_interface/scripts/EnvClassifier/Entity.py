from enum import Enum
from collections import OrderedDict

class Entity(object):

    def __init__(self, id):
        self.entity_id = id
        self.enhanced = False

    def show(self, plt):
        pass

    def getRepresentativePoint(self):
        pass

    def enhance(self, cluster):
        pass

    def getInfo(self):
        pass

    def isEnhanced(self):
        return self.enhanced

    def getId(self):
        return self.entity_id


class Wall(Entity):

    def __init__(self, id, polygon, segments, width):
        self.entity_id = id
        self.enhanced = False
        self.polygon = polygon
        self.segments = segments
        self.width = width
        self.length = None

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id), 
                            ("Name", "Wall"), 
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.enhanced), 
                            ("SegmentsCount", len(self.segments)), 
                            ("Width", round(self.getWidth(), 3)), 
                            ("Length", round(self.getLength(), 3))])

    def show(self, plt):
        for segment in self.segments:
            points = segment.coords
            plt.plot([p[0] for p in points], [p[1] for p in points], 'ro-')

    def enhance(self, cluster):
        pass

    def getWidth(self):
        return self.width

    def getLength(self):
        if self.length is None:
            self.length = 0
            for segment in self.segments:
                self.length += segment.length
        return self.length

    def getRepresentativePoint(self):
        return self.polygon.representative_point()


class Furniture(Entity):

    def __init__(self, id, polygon):
        self.entity_id = id
        self.polygon = polygon
        self.type = None
        self.is_enhanced = False

    def enhance(self, cluster):
        return self, cluster

    def getRepresentativePoint(self):
        return self.polygon.representative_point()

    def show(self, plt):
        pass

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id), 
                            ("Name", "Furniture"), 
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced",  self.is_enhanced), 
                            ("Type", self.type)])


class Door(Entity):

    class DoorState(Enum):
        OPEN = 0
        CLOSED = 1

    def __init__(self, id, segment):
        self.entity_id = id
        self.state = DoorState.CLOSED
        self.is_enhanced = False
        self.segment = segment

    def getState(self):
        return self.state

    def getLocation(self):
        return self.getCenter()

    def getRepresentativePoint(self):
        return self.segment.centroid

    def enhance(self, cluster):
        pass

    def show(self, plt):
        pass

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id), 
                            ("Name", "Door"), 
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.is_enhanced), 
                            ("Length", round(self.segment.length, 3)), 
                            ("State", self.state)])


class Noise(Entity):

    def __init__(self, id, polygon):
        self.entity_id = id
        self.center = None
        self.polygon = polygon
        self.is_enhanced = False

    def getRepresentativePoint(self):
        return self.polygon.representative_point()

    def enhance(self, cluster):
        pass

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id), 
                            ("Name", "Noise"), 
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.is_enhanced), 
                            ("Area", round(self.polygon.area, 3))])

    def show(self, plt):
        pass