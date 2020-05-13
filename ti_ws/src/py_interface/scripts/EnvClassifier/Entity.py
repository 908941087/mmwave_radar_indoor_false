from enum import Enum
from collections import OrderedDict
from ShowShapes import *


class Entity(object):

    def __init__(self, eid):
        self.entity_id = eid

    def show(self, plt):
        pass

    def getRepresentativePoint(self):
        pass

    def getInfo(self):
        pass

    def getId(self):
        return self.entity_id


class Wall(Entity):

    def __init__(self, eid, polygon, segments, width):
        super(Wall, self).__init__(eid)
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

    def showShape(self, plt):
        showPolygon(self.polygon, plt)

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

    def getPolygon(self):
        return self.polygon

    def getRepresentativePoint(self):
        return self.polygon.representative_point()


class Furniture(Entity):

    def __init__(self, eid, polygon):
        super(Furniture, self).__init__(eid)
        self.polygon = polygon
        self.type = None
        self.is_enhanced = False

    def enhance(self, cluster):
        return self, cluster

    def getRepresentativePoint(self):
        return self.polygon.representative_point()

    def show(self, plt):
        pass

    def showShape(self, plt):
        showPolygon(self.polygon, plt)

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id),
                            ("Name", "Furniture"),
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.is_enhanced),
                            ("Type", self.type)])


class DoorState(Enum):
    OPEN = 0
    CLOSED = 1


class Door(Entity):

    def __init__(self, eid, segment):
        super(Door, self).__init__(eid)
        self.state = DoorState.CLOSED
        self.is_enhanced = False
        self.segment = segment

    def getState(self):
        return self.state

    def getRepresentativePoint(self):
        return self.segment.centroid

    def getLength(self):
        return self.segment.length

    def enhance(self, cluster):
        pass

    def show(self, plt):
        self.showShape(plt)

    def showShape(self, plt):
        showLineString(self.segment, plt)

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id),
                            ("Name", "Door"),
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.is_enhanced),
                            ("Length", round(self.segment.length, 3)),
                            ("State", self.state)])


class Noise(Entity):

    def __init__(self, eid, polygon):
        super(Noise, self).__init__(eid)
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

    def showShape(self, plt):
        showPolygon(self.polygon, plt)
