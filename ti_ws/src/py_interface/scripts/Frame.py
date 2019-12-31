from sensor_msgs.msg import PointCloud2, std_msgs
from sensor_msgs import point_cloud2


class Point:

    def __init__(self, x=0, y=0, z=0, intensity=40):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y)

    def __repr__(self):
        return self.__str__()

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Point(x, y)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Point(x, y)

    def __truediv__(self, other):
        x = self.x / other
        y = self.y / other
        return Point(x, y)

    def __mul__(self, other):
        x = self.x * other
        y = self.y * other
        return Point(x, y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class Frame:

    def __init__(self, data):
        self.index = -1
        self.points = []
        self.fields = data.fields
        self.height = 1
        self.width = 0
        self.length = 0
        try:
            gen = point_cloud2.read_points(data)
            self.header = data.header
            for p in gen:
                self.append(Point(p[0], p[1], p[2], p[3]))
                self.width += 1
        except TypeError:
            pass

    def generate(self):
        header = self.header
        points = [(i.x, i.y, i.z, i.intensity) for i in self.points]
        return point_cloud2.create_cloud(header, self.fields, points)

    def append(self, point):
        self.points.append(point)
        self.length += 1

    def __next__(self):
        if self.index == self.length - 1:
            raise StopIteration
        else:
            self.index += 1
            return self.points[self.index]

    def __iter__(self):
        self.index = -1
        return self

    def __repr__(self):
        s = ""
        for p in self.points:
            s += str(p)
        return s

    def __str__(self):
        return self.__repr__()

    def __getitem__(self, key):
        return self.points[key]

    def __len__(self):
        return self.length