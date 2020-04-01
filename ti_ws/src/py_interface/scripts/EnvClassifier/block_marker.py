# coding=utf-8
import numpy as np
from utils import get_points_from_pcd, get_xy_lim, filter_points, get_slope, k2slope, get_area
import math
from math import ceil
from linear_regressor import LinearRegressor

"""
block_marker.py 主要用来实现对地图点分网格进行标记
"""

class BlockMarkerInterface:

    """
    需要实现这个接口，来对点进行标记
    """
    LOWEST_POINTS_COUNT = 10 # 若Block中的点的数量少于这个，则将其忽略，不参与下一步的divide

    def __init__(self, resolution=1):
        """
        @param resolution: int, in meters, the length of a block's edge
        """
        self.resolution = float(resolution)
        self.origin = []

    def set_points(self, points):
        """
        需要被标记的点集
        """
        self.points = points

    def mark(self):
        """
        To be Implemented by subclasses
        @return: 包含Block的二维数组
        """
        pass

    def pos2cordinate(self, pos):
        """
        Convert position of the block to coordinate system
        @pos: position of the block in the 2d array
        """
        return [pos[0] * self.resolution + self.origin[0], pos[1] * self.resolution + self.origin[1]]


class BlockMarker(BlockMarkerInterface):

    def mark(self):
        blocks = []
        xy_lim = get_xy_lim(self.points) # [x_min, x_max, y_min, y_max]
        self.origin = [xy_lim[0], xy_lim[2]]
        row = int(ceil((xy_lim[3] - xy_lim[2]) / self.resolution))
        col = int(ceil((xy_lim[1] - xy_lim[0]) / self.resolution))
        reg = LinearRegressor()
        for i in range(row):
            row_blocks = []
            for j in range(col):
                position = [i, j]
                cordinate = self.pos2cordinate(position)
                points = filter_points(self.points, cordinate[0], cordinate[0] + self.resolution, \
                    cordinate[1], cordinate[1] + self.resolution)
                if len(points) < self.LOWEST_POINTS_COUNT: # very few points in the block, param is None
                    row_blocks.append(Block(None, points, position))
                else:
                    reg.process(points)
                    param = reg.get_parameters()[0]
                    slope = k2slope(param[0])
                    row_blocks.append(Block(param, points, position))
            blocks.append(row_blocks)
        return blocks


class Block:

    """
    是BlockMarker的输出，包含一个网格中的数据点，网格的位置（实际上是网格在二维数组中的二维索引），网格中的点拟合线的参数
    """

    def __init__(self, param, points, position):
        """
        @param param: [k, b], y = k * x + b
        @param points: 网格中的数据点；若Block为空，则points=[]
        @param position: 网格的二维索引, [i， j]
        """
        self.param = param
        self.points = points
        self.position = position
        self.center = None
        self.density = None

    def get_center(self):
        if self.center is None: 
            self.center = [np.average([p[0] for p in self.points]), np.average([p[1] for p in points])]
        return self.center

    def get_density(self):
        if self.density is None: 
            return len(self.points) / get_area(self.points)
        return self.density

    def get_slope(self):
        return k2slope(self.param[0])

    def get_intersections(self):
        k = self.param[0]
        b = self.param[1]
        xy_lim = get_xy_lim(self.points)
        if k == 0: 
            return [[xy_lim[0], b], [xy_lim[1], b]]
        elif k == float('inf') and b != float('inf'):
            return [[b, xy_lim[2]], [b, xy_lim[3]]]
        else:
            x = [xy_lim[0], xy_lim[1], (xy_lim[2] - b) / k, (xy_lim[3] - b) / k]
            x.sort()
            return [[i, k * i + b] for i in x if x[0] < i < x[3]]

    def __str__(self):
        return str(self.position) + ": " + str(self.slope)

    def __repr__(self):
        return str(self)

if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    points = get_points_from_pcd("four_walls.pcd")
    
    # Config figure
    padding = 0.2
    xy_lim = get_xy_lim(points)
    ratio = (xy_lim[3] - xy_lim[2]) / float(xy_lim[1] - xy_lim[0])
    fig = plt.figure(figsize=(10 * ratio, 10))
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.grid(True, linewidth=0.5, color='#666666', linestyle='dotted')
    ax1.axis([xy_lim[0] - padding, xy_lim[1] + padding, xy_lim[2] - padding, xy_lim[3] + padding])
    ax1.set_color_cycle(['red', 'black', 'blue', 'brown', 'green'])
    # ax1.scatter([p[0] for p in points], [p[1] for p in points], color='red', s=1)

    # Marking
    resolution = 0.5
    marker = BlockMarker(resolution)
    marker.set_points(points)
    blocks = marker.mark()
    
    # Plot
    clamped = False # clamped为TRUE，则使用0，45，90，135度
    for row in blocks:
        for block in row:
            if block.param is None:
                ax1.scatter([p[0] for p in block.points], [p[1] for p in block.points], s=1)
                continue
            else:
                intersections = []
                if clamped:
                    cordinate = marker.pos2cordinate(block.position)
                    slope = int(round(block.get_slope() / 45.0)) * 45 % 180
                    if slope == 0:
                        intersections = [[cordinate[0], cordinate[1] + resolution / 2], \
                            [cordinate[0] + resolution, cordinate[1] + resolution / 2]]
                    elif slope == 45:
                        intersections = [cordinate, [cordinate[0] + resolution, cordinate[1] + resolution]]
                    elif slope == 90:
                        intersections = [[cordinate[0] + resolution / 2, cordinate[1]], \
                            [cordinate[0] + resolution / 2, cordinate[1] + resolution]]
                    elif slope == 135:
                        intersections = [[cordinate[0], cordinate[1] + resolution], \
                            [cordinate[0] + resolution, cordinate[1]]]
                else: 
                    intersections = block.get_intersections()
                ax1.scatter([p[0] for p in block.points], [p[1] for p in block.points], s=1)
                ax1.plot([i[0] for i in intersections], [i[1] for i in intersections], linewidth=1)

    plt.show()
