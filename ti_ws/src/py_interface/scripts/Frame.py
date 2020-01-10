from sensor_msgs.msg import PointCloud2, std_msgs
from sensor_msgs import point_cloud2
from math import sqrt, pow
from collections import deque


class Point:

    def __init__(self, x=0, y=0, z=0, intensity=40):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " z: " + str(self.z)

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

    def dist(self, other):
        return sqrt(pow((self.x - other.x), 2) + pow((self.y - other.y), 2))

class Frame:

    def __init__(self):
        self.index = -1
        self.points = []
        self.fields = None
        self.header = None
        self.height = 1
        self.width = 0
        self.length = 0
        
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

class StabilityMap:
    resolution = 5  # 5cm

    def __init__(self, col, row):
        self.col = col
        self.row = row
        self.stability_map = [[0 for i in range(col)] for j in range(row)]

    def add_frame(self, frame):
        for p in frame:
            x, y = self.get_correspond_block(p)
            self.stability_map[x][y] += p.intensity

    def subtract_frame(self, frame):
        for p in frame:
            x, y = self.get_correspond_block(p)
            self.stability_map[x][y] -= p.intensity
    
    def translate_axis(self, point):
        x = point.x + self.col / 2
        y = -point.y + self.row
        return x, y

    def get_correspond_block(self, point):
        x, y = self.translate_axis(point)
        x = int(x / self.resolution)
        y = int(y / self.resolution)
        return x, y

    def get_correspond_neighbors(self, point):
        # get its 4 nearest blocks
        def sgn(x):
            if x == 0:
                return 0
            return int(x / abs(x))

        def clamp(x, low, high):
            if x < low:
                x = low
            if x > high:
                x = high
            return x

        x, y = self.translate_axis(point)
        block_x, block_y = self.get_correspond_block(point)
        diff_x = point.x - (block_x * self.resolution + self.resolution / 2)
        diff_y = point.y - (block_y * self.resolution + self.resolution / 2)
        neighbors = []
        neighbors.append([block_x, block_y])
        neighbors.append([clamp(block_x + sgn(diff_x), 0, self.col), block_y])
        neighbors.append([block_x, clamp(block_y + sgn(diff_y), 0, self.row)])
        neighbors.append([clamp(block_x + sgn(diff_x), 0, self.col), clamp(block_y + sgn(diff_y), 0, self.row)])
        return neighbors

    def get_correspond_stability(self, point):
        # get the stability value for point using 4 nearest blocks
        neighbors = self.get_correspond_neighbors(point)
        weights = [0.5, 0.2, 0.2, 0.1]
        result = 0
        for neighbor, weight in zip(neighbors, weights):
            result += self.stability_map[neighbor.x][neighbor.y] * weight
        return result

    def filter_frame(self, frame, threshold=0):  # Set threshold to achieve different out rate
        f = Frame()
        f.header = frame.header
        for point in frame:
            stability = self.get_correspond_stability(point)
            if stability >= threshold:
                f.append(point)
        return f


class Blocks(Stability_map):
    resolution = 5  # 5cm

    def __init__(self, col, row):
        self.col = col
        self.row = row
        self.blocks = [[[] for i in range(col)] for j in range(row)]

    def add_frame(self, frame):
        for p in frame:
            x, y = self.get_correspond_block(p)
            self.blocks[x][y].append(p)

    def subtract_frame(self, frame):
        for p in frame:
            x, y = self.get_correspond_block(p)
            self.blocks[x][y].remove(p)

    def get_correspond_stability(self, point):
        neighbors = self.get_correspond_neighbors(point)
        stability = 0
        for n in neighbors:
            for p in n:
                stability += p.intensity / point.dist(p)
        return stability


class FrameService:
    def __init__(self):
        pass

    def get_frames_from_file(self, path):
        try:
            frames = []
            with open(path, 'r') as f:
                lines = f.readlines()
                temp_frame = Frame()
                current_frame_number = 0
                for line in lines:
                    if not len(line) == 0:
                        temp = eval(line)
                        if not current_frame_number == temp.frame:
                            frames.append(temp_frame)
                            temp_frame = Frame()
                        temp_frame.append(Point(temp.x, temp.y))
                frames.append(temp_frame)  # the last frame
            return frames
        except Exception:
            print("Could not parse file.")

    def point_cloud_to_frame(self, data):
        f = Frame()
        try:
            gen = point_cloud2.read_points(data)
            f.header = data.header
            for p in gen:
                f.append(Point(p[0], p[1], p[2], p[3]))
                f.width += 1
            return f
        except TypeError:
            print("Could not parse point_cloud2.")

    def frame_to_point_cloud(self, frame):
        header = frame.header
        points = [(i.x, i.y, i.z, i.intensity) for i in frame.points]
        return point_cloud2.create_cloud(header, frame.fields, points)

    def find_neighbors(self, source_frame, reference_frame):
        try:
            match_data_frame = []
            for point in source_frame:
                match_data_frame.extend(self.find_match_in_frame(point, reference_frame))
            f = Frame()
            f.header = source_frame.header
            f.points = match_data_frame
            return f
        except TypeError as e:
            return source_frame

    def find_match_in_frame(self, source_point, frame):
        neighbor_threshold = 0.1  # FIXME: need revise
        match = []  # list to store matched points in a frame
        for p in frame:
            if p.dist(source_point) <= neighbor_threshold:
                match.append(p)
        return match

    def get_multi_frame_stablizer(self, n):
        class MultiFrameStablizer:
            def __init__(self, n):
                self.frames = deque(maxlen=n)
                for i in range(n):
                    self.frames.append(Frame())

                col = int(10 * 100 / Blocks.resolution)
                row = int(5 * 100 / Blocks.resolution)
                self.blocks = Blocks(col, row)
            
            def update(self, frame):
                self.blocks.subtract_frame(self.frames[0])
                self.blocks.add_frame(self.frames[len(self.frames) - 1])
                self.frames.popleft()
                self.frames.append(frame)
                return self.blocks.filter_frame(frame)

        return MultiFrameStablizer(n)