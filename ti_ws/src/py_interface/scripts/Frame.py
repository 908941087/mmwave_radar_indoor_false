# from sensor_msgs.msg import PointCloud2, std_msgs
# from sensor_msgs import point_cloud2
from math import sqrt, pow, floor
import numpy as np
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
        return (self.x == other.x and 
                self.y == other.y and 
                self.z == other.z and 
                self.intensity == other.intensity)

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

    def next(self):
        if self.index == self.length - 1:
            raise StopIteration
        else:
            self.index += 1
            return self.points[self.index]

    def __iter__(self):
        self.index = -1
        return self

    def __repr__(self):
        s = "["
        for p in self.points:
            s += "    " + str(p)
        return s + "]"

    def __str__(self):
        return self.__repr__()

    def __getitem__(self, key):
        return self.points[key]

    def __len__(self):
        return self.length

class Blocks():
    resolution = 1

    def __init__(self, col, row):
        self.col = col
        self.row = row

    def translate_axis(self, point):
        x = point.x + self.resolution * (self.col / 2)
        y = -point.y + self.row * self.resolution
        return x, y

    def get_block(self, point):
        x, y = self.translate_axis(point)
        if not (0 <= x < self.col * self.resolution and 0 <= y < self.row * self.resolution):
            raise IndexError("Point is not inside the map.")
        x = int(floor(x / self.resolution))
        y = int(floor(y / self.resolution))

        return x, y

    def get_neighbors(self, point):
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
        block_x, block_y = self.get_block(point)
        diff_x = x - (block_x * self.resolution + self.resolution / 2)
        diff_y = y - (block_y * self.resolution + self.resolution / 2)
        neighbors = []
        neighbors.append([block_x, block_y])
        neighbors.append([int(clamp(block_x + sgn(diff_x), 0, self.col - 1)), block_y])
        neighbors.append([block_x, int(clamp(block_y + sgn(diff_y), 0, self.row - 1))])
        neighbors.append([int(clamp(block_x + sgn(diff_x), 0, self.col - 1)), int(clamp(block_y + sgn(diff_y), 0, self.row - 1))])
        return neighbors

    def get_stability(self, point):
        pass

    def filter_frame(self, frame, threshold=0):  # Set threshold to achieve different out rate
        f = Frame()
        f.header = frame.header
        f.fields = frame.fields
        for point in frame:
            stability = self.get_stability(point)
            if stability >= threshold:
                f.append(point)
                f.width += 1
        return f
    

class BlockStabilityMap(Blocks):
    resolution = 5  # 5cm

    def __init__(self, col, row):
        self.col = col
        self.row = row
        self.stability_map = [[0 for i in range(col)] for j in range(row)]

    def add_frame(self, frame):
        for p in frame:
            x, y = self.get_block(p)
            self.stability_map[x][y] += p.intensity

    def subtract_frame(self, frame):
        for p in frame:
            x, y = self.get_block(p)
            self.stability_map[x][y] -= p.intensity

    def get_stability(self, point):
        # get the stability value for point using 4 nearest blocks
        neighbors = self.get_neighbors(point)
        weights = [0.5, 0.2, 0.2, 0.1]
        result = 0
        for neighbor, weight in zip(neighbors, weights):
            result += self.stability_map[neighbor.x][neighbor.y] * weight
        return result


class HitMap(Blocks):
    resolution = 5  # 5cm

    def __init__(self, col, row):
        self.col = col
        self.row = row
        self.blocks = [[[] for i in range(col)] for j in range(row)]

    def add_frame(self, frame):
        for p in frame:
            x, y = self.get_block(p)
            self.blocks[x][y].append(p)

    def subtract_frame(self, frame):
        for p in frame:
            x, y = self.get_block(p)
            self.blocks[x][y].remove(p)

    def get_stability(self, point):
        neighbors = self.get_neighbors(point)
        stability = 0
        for n in neighbors:
            for p in self.blocks[n[0]][n[1]]:
                if not p == point:
                    stability += p.intensity / pow(point.dist(p), 2)
        return stability

class TimeStabilityMap(Blocks):
    resolution = 2

    def __init__(self, col, row, n):
        self.col = col
        self.row = row
        self.variances = [[None for i in range(col)] for j in range(row)]
        self.blocks = deque(maxlen=n)
        for i in range(n):
            temp = BlockStabilityMap(self.col, self.row)
            temp.resolution = self.resolution
            self.blocks.append(temp)

    def update(self, frame):
        self.blocks.popleft()
        block = BlockStabilityMap(self.col, self.row)
        block.resolution = self.resolution
        block.add_frame(frame)
        self.blocks.append(block)
        self.variances = [[None for i in range(self.col)] for j in range(self.row)]

    def get_variance(self, x, y):
        if self.variances[x][y] is None:
            # print([i.stability_map[x][y] for i in self.blocks])
            self.variances[x][y] = np.var([i.stability_map[x][y] for i in self.blocks])
        return self.variances[x][y]

    def filter_frame(self, frame, threshold=0):
        f = Frame()
        f.header = frame.header
        f.fields = frame.fields
        for p in frame:
            x, y = self.get_block(p)
            variance = self.get_variance(x, y)
            # print(variance)
            if variance <= threshold:
                f.append(p)
        return f


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
                    if not len(line) == 1:
                        temp = eval(line)
                        if not current_frame_number == temp['frame'] and not temp_frame.length == 0:
                            frames.append(temp_frame)
                            # print(temp_frame)
                            temp_frame = Frame()
                        current_frame_number = temp['frame']
                        temp_frame.append(Point(temp['x'], temp['y']))
                frames.append(temp_frame)  # the last frame
            return frames
        except Exception:
            print("Could not parse file.")
            raise Exception

    def point_cloud_to_frame(self, data):
        f = Frame()
        try:
            gen = point_cloud2.read_points(data)
            f.header = data.header
            f.fields = data.fields
            for p in gen:
                f.append(Point(p[1], p[0], p[2], p[3]))
                f.width += 1
            return f
        except TypeError:
            print("Could not parse point_cloud2.")
            raise TypeError

    def frame_to_point_cloud(self, frame):
        header = frame.header
        points = [(i.y, i.x, i.z, i.intensity) for i in frame]
        return point_cloud2.create_cloud(header, frame.fields, points)

    def find_neighbors(self, source_frame, reference_frame):
        try:
            match_data_frame = []
            for point in source_frame:
                match_data_frame.extend(self.find_match_in_frame(point, reference_frame))
            f = Frame()
            f.header = source_frame.header
            f.fields = source_frame.fields
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

    def get_multi_frame_stablizer(self, width=10, height=10, resolution=1, frame_num=5, threshold=0):
        class MultiFrameStablizer:
            def __init__(self, width, height, resolution, frame_num, threshold):
                # self.frames = deque(maxlen=frame_num)
                self.width = width
                self.height = height
                self.resolution = resolution
                self.frame_num = frame_num
                self.threshold = threshold
                # for i in range(self.frame_num):
                #     self.frames.append(Frame())

                col = int(self.width * 100 / self.resolution)
                row = int(self.height * 100 / self.resolution)
                self.time_stability_map = TimeStabilityMap(col, row, frame_num)
                self.time_stability_map.resolution = self.resolution
            
            def update(self, frame):
                # self.frames.popleft()
                # self.frames.append(frame)
                temp = self.time_stability_map.filter_frame(frame, self.threshold)  # Change this to achieve different out rate
                self.time_stability_map.update(frame)
                return temp

        return MultiFrameStablizer(width, height, resolution, frame_num, threshold)

if __name__ == '__main__':
    frame_service = FrameService()

    frames = frame_service.get_frames_from_file("data.txt")

    stablizer = frame_service.get_multi_frame_stablizer(width=10, height=10, resolution=0.5, frame_num=5, threshold=10000)

    for frame in frames:
        stable_frame = stablizer.update(frame)
        print(stable_frame.length, frame.length)
        print(stable_frame.length / float(frame.length))