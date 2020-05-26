from enum import Enum
from collections import OrderedDict
from ShapeOperator.ShapeViewer import *
from ShapeOperator.PointsGenerator import generateForPolygon
from Cluster import Cluster, ClusterType
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Polygon, Point
from numpy import sqrt



class Entity(object):

    def __init__(self, eid):
        self.entity_id = eid

    def show(self, plt):
        pass

    def getRepresentativePoint(self):
        pass

    def getInfo(self):
        pass

    def getInfoMarker(self, mark_index, duration):
        t_marker = Marker()
        t_marker.header.frame_id = "/map"
        t_marker.header.stamp = rospy.Time.now()
        t_marker.ns = "cluster_class"

        t_marker.id = mark_index
        # Type
        t_marker.type = Marker.TEXT_VIEW_FACING
        loc = self.getRepresentativePoint()
        if loc is None: print(self.getId())
        t_marker.text = self.getInfo()["Name"] + "\n" + "x: " + str(round(loc.x, 3)) + "\n" + "y: " + str(
            round(loc.y, 3))
        # Size
        t_marker.scale.x = 0.2
        t_marker.scale.y = 0.2
        t_marker.scale.z = 0.2

        # ADD/DELETE
        t_marker.action = Marker.MODIFY

        # Pose
        t_marker.pose.position.x = loc.x
        t_marker.pose.position.y = loc.y
        t_marker.pose.position.z = 0.2
        t_marker.pose.orientation.x = 0.0
        t_marker.pose.orientation.y = 0.0
        t_marker.pose.orientation.z = 0.0
        t_marker.pose.orientation.w = 1.0

        # Color
        t_marker.color.r = 0.0
        t_marker.color.g = 1.0
        t_marker.color.b = 0.5
        t_marker.color.a = 1.0

        t_marker.lifetime = rospy.Duration(duration)
        return t_marker

    def getId(self):
        return self.entity_id

    def getShapeMarker(self, mark_index, duration):
        pass


class Wall(Entity):

    def __init__(self, eid, polygon, length, width):
        super(Wall, self).__init__(eid)
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
        return Wall(self.getId(), poly, length, area / length), c

    def getWidth(self):
        return self.width

    def getLength(self):
        return self.length

    def getPolygon(self):
        return self.polygon

    def getShapeMarker(self, mark_index, duration):
        p_marker = Polygon()
        vertices = self.polygon.coords
        for v in vertices:
            p_marker.points.append(Point(v[0], v[1]))
        return p_marker

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

    def __init__(self, eid, polygon):
        super(UnfinishedEntity, self).__init__(eid)
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
