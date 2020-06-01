from enum import Enum
from collections import OrderedDict
from ShapeOperator.ShapeViewer import *
from ShapeOperator.PointsGenerator import generateForPolygon
from Cluster import Cluster, ClusterType
import rospy
from numpy import sqrt


class Entity(object):

    def __init__(self):
        self.entity_id = None
        self.height = 0
        self.is_valid = False

    def setHeight(self, height):
        self.height = height

    def getHeight(self):
        return self.height

    def setId(self, eid):
        self.entity_id = eid
        self.is_valid = True

    def show(self, plt):
        pass

    def getRepresentativePoint(self):
        pass

    def getInfo(self):
        pass

    def getId(self):
        return self.entity_id

    def getShapeMarker(self, mark_index, duration):
        pass


class Wall(Entity):

    def __init__(self, polygon, length, width):
        super(Wall, self).__init__()
        self.enhanced = False
        self.polygon = polygon
        self.length = length
        self.width = width

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id),
                            ("Name", "W"),
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.enhanced),
                            ("Width", round(self.getWidth(), 3)),
                            ("Length", round(self.getLength(), 3))])

    def show(self, plt):
        self.showShape(plt)

    def showShape(self, plt):
        showPolygon(self.polygon, plt)

    def enhance(self, cluster):
        poly = self.getPolygon().simplify(tolerance=0.2, preserve_topology=False)
        points = generateForPolygon(poly)
        c = Cluster(self.getId(), ClusterType.LASER, 2, points)
        perimeter = poly.length
        area = poly.area
        length = perimeter / 4.0 + sqrt(perimeter ** 2 / 4.0 - 4 * area) / 2.0
        wall = Wall(poly, length, area / length)
        return wall, c

    def getWidth(self):
        return self.width

    def getLength(self):
        return self.length

    def getPolygon(self):
        return self.polygon

    def getRepresentativePoint(self):
        return self.polygon.representative_point()


class Furniture(Entity):

    def __init__(self, polygon):
        super(Furniture, self).__init__()
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

    def getPolygon(self):
        return self.polygon

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id),
                            ("Name", "F"),
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.is_enhanced),
                            ("Type", self.type)])


class DoorState(Enum):
    OPEN = 0
    CLOSED = 1


class TranspanrentObstacle(Entity):

    def __init__(self, segment):
        super(TranspanrentObstacle, self).__init__()
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

    def getSegment(self):
        return self.segment

    def showShape(self, plt):
        showLineString(self.segment, plt, 'go-')

    def getInfo(self):
        loc = self.getRepresentativePoint()
        return OrderedDict([("Id", self.entity_id),
                            ("Name", "GD"),
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.is_enhanced),
                            ("Length", round(self.segment.length, 3)),
                            ("State", self.state)])


class UnfinishedEntity(Entity):

    def __init__(self, polygon):
        super(UnfinishedEntity, self).__init__()
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
                            ("Name", "UE"),
                            ("Location", [round(loc.x, 2), round(loc.y, 2)]),
                            ("Enhanced", self.is_enhanced),
                            ("Area", round(self.polygon.area, 3))])

    def show(self, plt):
        pass

    def showShape(self, plt):
        showPolygon(self.polygon, plt)

    def getPolygon(self):
        return self.polygon
