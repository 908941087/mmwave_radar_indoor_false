from utils import get_xy_lim
from math import ceil, floor
import numpy as np

class MorphologyOperator(object):
    def __init__(self, step_length=0.1):
        self.step_length = step_length

    def rasterize(self, cluster):
        if cluster is None: raise ValueError("cluster must be specified.")
        xy_lim = get_xy_lim(cluster)

        def get_cell(point):
            x_offset = point[0] - xy_lim[0]
            y_offset = point[1] - xy_lim[2]
            return int(floor(y_offset / self.step_length)), int(floor(x_offset / self.step_length))

        row_count = int(ceil((xy_lim[3] - xy_lim[2]) / self.step_length))
        col_count = int(ceil((xy_lim[1] - xy_lim[0]) / self.step_length))
        if row_count == 0: row_count = 1
        if col_count == 0: col_count = 1
        grid = [[0 for i in range(col_count)] for j in range(row_count)]
        for p in cluster:
            i, j = get_cell(p)
            try:
                grid[i][j] = 1
            except IndexError:
                pass
        return grid, xy_lim

    def derasterize(self, grid, xy_lim):
        cluster = []
        offset = self.step_length / 2.0

        def get_point(cell):
            y = cell[0] * self.step_length + xy_lim[2] - offset
            x = cell[1] * self.step_length + xy_lim[0] - offset
            return [x, y]

        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    cluster.append(get_point([i, j]))
        return cluster

    @staticmethod
    def filter(grid, kernel_size, operator, initial_value):
        row = len(grid)
        col = len(grid[0])
        new_grid = [[0 for i in range(col)] for j in range(row)]
        padding = kernel_size>>1

        def get_cell_value(cell):
            result = initial_value
            for i in range(cell[0] - padding, cell[0] + padding):
                for j in range(cell[1] - padding, cell[1] + padding):
                    result = operator(result, grid[i][j])
            return result

        for i in range(row):
            for j in range(col):
                if padding <= i < row - padding and padding <= j < col - padding:
                    new_grid[i][j] = get_cell_value([i, j])
                else:
                    new_grid[i][j] = grid[i][j]
        return new_grid
        
    def erosion(self, cluster, kernel_size=5):
        grid, xy_lim = self.rasterize(cluster)
        
        def operator(a, b):
            return a&b

        grid = self.filter(grid, kernel_size, operator, 1)
        return self.derasterize(grid, xy_lim)

    def dilation(self, cluster, kernel_size=5):
        grid, xy_lim = self.rasterize(cluster)

        def operator(a, b):
            return a|b

        grid = self.filter(grid, kernel_size, operator, 0)
        return self.derasterize(grid, xy_lim)

    def opening(self, cluster, kernel_size=5):
        grid, xy_lim = self.rasterize(cluster)

        def logical_and_operator(a, b):
            return a&b
        
        def logical_or_operator(a, b):
            return a|b

        grid = self.filter(grid, kernel_size, logical_and_operator, 1)
        grid = self.filter(grid, kernel_size, logical_or_operator, 0)
        return self.derasterize(grid, xy_lim)

    def closing(self, cluster, kernel_size=5):
        grid, xy_lim = self.rasterize(cluster)

        def logical_and_operator(a, b):
            return a&b
        
        def logical_or_operator(a, b):
            return a|b

        grid = self.filter(grid, kernel_size, logical_or_operator, 0)
        grid = self.filter(grid, kernel_size, logical_and_operator, 1)
        return self.derasterize(grid, xy_lim)


if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from utils import get_points_from_pcd

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(1, 1, 1)
    ax.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')

    points = get_points_from_pcd("ti_ws/src/py_interface/scripts/EnvClassifier/pcds/3d_pc_map.pcd")
    
    ax.scatter([p[0] for p in points], [p[1] for p in points], c='r', s=1)

    mo = MorphologyOperator(0.1)
    points_after_erosion = mo.closing(points, 3)
    ax.scatter([p[0] for p in points_after_erosion], [p[1] for p in points_after_erosion], c='b', s=1)
    plt.show()