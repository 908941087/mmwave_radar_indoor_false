import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class ColorGenerator:

    def __init__(self):
        self.index = 0
        self.color_list = []
        self.generateList()

    def generateList(self):
        rgb_list = [
            [255, 51, 153],
            [153, 51, 255],
            [51, 153, 255],
            [51, 255, 153],
            [153, 255, 51],
            [255, 153, 51],
        ]
        for rgb in rgb_list:
            self.color_list.append(struct.unpack('I', struct.pack('BBBB', rgb[2], rgb[1], rgb[0], 0))[0])

    def getColor(self):
        c = self.color_list[self.index]
        self.index = (self.index + 1) % len(self.color_list)
        return c


def ShapelyPointToROSPoint(p, color=None):
    return [p.x, p.y, p.z, color]


def MultiPointToPoints(mp, color=None):
    return [ShapelyPointToROSPoint(p, color) for p in mp]


def ClustersToPointCloud2(clusters):
    color_generator = ColorGenerator()

    points = []
    for c in clusters:
        points.extend(MultiPointToPoints(c.getPoints(), color_generator.getColor()))

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.UINT32, 1),
              # PointField('rgba', 12, PointField.UINT32, 1),
              ]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()
    pc2 = point_cloud2.create_cloud(header, fields, points)

    return pc2
