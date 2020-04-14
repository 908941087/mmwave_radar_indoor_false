from sklearn.neighbors import KDTree
import numpy as np
from utils import dist, k2slope
from collections import deque
from wall_finder import WallFinder

class WallLinker(object):
    WALL_LINK_THRESHOLD = 0.5
    SAME_DIRECTION_THRESHOLD = 45 # degrees

    def __init__(self):
        pass

    def link_fit(self, clusters, walls):
        if (len(clusters) == 0): raise Exception("No clusters available.")
        wall_centers = [[(wall["ends"][0][0] + wall["ends"][1][0]) / 2.0, (wall["ends"][0][1] + wall["ends"][1][1]) / 2.0] for wall in walls]
        if len(wall_centers) in [0, 1]: return
        wall_centers = np.array(wall_centers).reshape(-1, 2)
        tree = KDTree(wall_centers, leaf_size=2)
        query_size = 6 # bigger means better accuracy, smaller means better speed
        if query_size > len(wall_centers): query_size = len(wall_centers)

        def is_connected(id1, id2):
            dists = []
            for p1 in walls[id1]["ends"]:
                for p2 in walls[id2]["ends"]:
                    dists.append(dist(p1, p2))
            dists.sort()
            near = dists[0] < self.WALL_LINK_THRESHOLD

            slope1 = k2slope(walls[id1]["line"][0])
            slope2 = k2slope(walls[id2]["line"][0])
            same_direction = min(abs(slope1 - slope2), 180 - abs(slope1 - slope2)) < self.SAME_DIRECTION_THRESHOLD

            return near and same_direction

        blocks_count = len(clusters)
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
                part.extend(clusters[cur])
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
        
        wall_finder = WallFinder()
        walls = []
        index = 0
        for part in parts:
            wall = wall_finder.fit_a_wall(part)
            wall["ID"] = index
            walls.append(wall)
            index += 1
        return walls