# coding=utf-8

from utils import *
from linear_regressor import LinearRegressor
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
from collections import deque

class WallFinder(object):
    WIDTH_THRESHOLD = 0.32
    LEAST_POINTS_COUNT_TO_FIND_WALL = 20
    WALL_LINK_THRESHOLD = 0.5
    SAME_DIRECTION_THRESHOLD = 45 # degrees

    def __init__(self):
        self.walls = []
        self.regressor = LinearRegressor()
        self.index = 0
        self.blocks = []

    def fit_a_wall(self, cluster):
        ends = self.regressor.process(cluster)
        line = self.regressor.get_parameters()[0]
        avg_dist = self.get_avg_distance(cluster, line)
        info = {"line": line, "ends":ends, "width": avg_dist * 2}
        return info

    def find_walls_core(self, cluster):
        if cluster is None or len(cluster) == 0:
            return None
        info = self.fit_a_wall(cluster)
        if info["width"] < self.WIDTH_THRESHOLD:
            self.blocks.append(cluster)
            info["ID"] = self.index
            self.walls.append(info)
            self.index += 1
        else:
            clusters = self.branch(cluster)
            for one_cluster in clusters:
                if len(one_cluster) > self.LEAST_POINTS_COUNT_TO_FIND_WALL: self.find_walls_core(one_cluster)
        return self.walls

    def find_walls(self, cluster):
        self.walls = []
        self.index = 0
        self.blocks = []
        self.walls = self.find_walls_core(cluster)
        parts = self.link_walls()
        if parts is None: return self.walls
        self.walls = []
        self.index = 0
        for p in parts:
            info = self.fit_a_wall(p)
            info["ID"] = self.index
            self.walls.append(info)
            self.index += 1
        return self.walls

    def link_walls(self):
        if (len(self.blocks) == 0): raise Exception("No blocks available, try to call set_blocks first.")
        wall_centers = [[(wall["ends"][0][0] + wall["ends"][1][0]) / 2.0, (wall["ends"][0][1] + wall["ends"][1][1]) / 2.0] for wall in self.walls]
        if len(wall_centers) in [0, 1]: return
        wall_centers = np.array(wall_centers).reshape(-1, 2)
        tree = KDTree(wall_centers, leaf_size=2)
        query_size = 6 # bigger means better accuracy, smaller means better speed
        if query_size > len(wall_centers): query_size = len(wall_centers)

        def is_connected(id1, id2):
            dists = []
            for p1 in self.walls[id1]["ends"]:
                for p2 in self.walls[id2]["ends"]:
                    dists.append(dist(p1, p2))
            dists.sort()
            near = dists[0] < self.WALL_LINK_THRESHOLD

            slope1 = k2slope(self.walls[id1]["line"][0])
            slope2 = k2slope(self.walls[id2]["line"][0])
            same_direction = min(abs(slope1 - slope2), 180 - abs(slope1 - slope2)) < self.SAME_DIRECTION_THRESHOLD

            return near and same_direction

        blocks_count = len(self.blocks)
        visited = set()
        unvisited = set([i for i in range(blocks_count)])
        edge = deque()
        parts = []
        start = 0
        while (len(visited) < blocks_count):
            part = []
            q = deque()
            cur = start
            q.append(cur)
            while len(q) != 0:
                cur = q.popleft()
                if cur in visited: raise Exception("Duplicate entry in the loop.")
                visited.add(cur)
                unvisited.remove(cur)
                part.extend(self.blocks[cur])
                distances, ind = tree.query(np.array(wall_centers[cur]).reshape(-1, 2), k=query_size)
                surroundings = ind[0][1:]
                for pos in surroundings:
                    if pos in visited:
                        continue
                    elif is_connected(cur, pos):
                        if pos not in q:
                            q.append(pos)
                        if pos in edge:
                            edge.remove(pos)
                    else:
                        if pos not in q and pos not in edge: 
                            edge.append(pos)
            if len(part) != 0: 
                parts.append(part)
            if len(edge) != 0:
                start = edge.popleft()
            else: 
                if len(unvisited) != 0: 
                    start = unvisited.pop()
                    unvisited.add(start)
                else: break
        return parts

    @staticmethod
    def branch(cluster):
        xy_lim = get_xy_lim(cluster)
        x_mid = (xy_lim[0] + xy_lim[1]) / 2.0
        y_mid = (xy_lim[2] + xy_lim[3]) / 2.0
        # plt.plot([x_mid, x_mid], [xy_lim[2], xy_lim[3]], c='r', linewidth=1, linestyle='dotted')
        # plt.plot([xy_lim[0], xy_lim[1]], [y_mid, y_mid], c='r', linewidth=1, linestyle='dotted')
        result = []
        result.append(filter_points(cluster, xy_lim[0], x_mid, xy_lim[2], y_mid))
        result.append(filter_points(cluster, x_mid, xy_lim[1], xy_lim[2], y_mid))
        result.append(filter_points(cluster, xy_lim[0], x_mid, y_mid, xy_lim[3]))
        result.append(filter_points(cluster, x_mid, xy_lim[1], y_mid, xy_lim[3]))
        return result

    @staticmethod
    def get_avg_distance(points, line):
        a = line[0]
        b = -1
        c = line[1]
        temp = sum([abs(a * p[0] + b * p[1] + c) for p in points])
        return temp / np.sqrt(a ** 2 + 1) / float(len(points))

    @staticmethod
    def prolong_ends(ends, ratio=1.1):
        """
        Prolong the ends according to the ratio
        """
        mid = [sum([p[0] for p in ends]) / 2.0, sum([p[1] for p in ends]) / 2.0]
        return [[mid[0] - (mid[0] - p[0]) * ratio, mid[1] - (mid[1] - p[1]) * ratio] for p in ends]
