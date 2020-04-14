from sklearn.neighbors import KDTree
import numpy as np
from utils import dist, k2slope, get_intersection
from collections import deque

class WallTrimmer(object):
    PERPENDICULAR_LOWER_THRESHOLD = 60 # degrees
    PERPENDICULAR_HIGHER_THRESHOLD = 120 # degrees
    WALL_LINK_THRESHOLD = 1.2 # meters

    def __init__(self):
        pass

    def trim(self, walls):
        if (len(walls) == 0): raise Exception("No blocks available, try to call set_blocks first.")
        wall_centers = [[(wall["ends"][0][0] + wall["ends"][1][0]) / 2.0, (wall["ends"][0][1] + wall["ends"][1][1]) / 2.0] for wall in walls]
        if len(wall_centers) in [0, 1]: return walls
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
            perpendicular = self.PERPENDICULAR_LOWER_THRESHOLD <= min(abs(slope1 - slope2), 180 - abs(slope1 - slope2)) <= self.PERPENDICULAR_HIGHER_THRESHOLD

            return near and perpendicular

        def trim_corner(id1, id2):
            wall1, wall2 = walls[id1], walls[id2]
            intersection = get_intersection(wall1["line"], wall2["line"])
            for wall in [wall1, wall2]:
                dists = [dist(p, intersection) for p in wall["ends"]]
                if (dists[0] < dists[1]):
                    wall["ends"][0] = intersection
                else:
                    wall["ends"][1] = intersection

        blocks_count = len(walls)
        visited = set()
        unvisited = set([i for i in range(blocks_count)])
        edge = deque()
        start = 0
        while (len(visited) < blocks_count):
            q = deque()
            cur = start
            q.append(cur)
            while len(q) != 0:
                cur = q.popleft()
                if cur in visited: raise Exception("Duplicate entry in the loop.")
                visited.add(cur)
                unvisited.remove(cur)
                distances, ind = tree.query(np.array(wall_centers[cur]).reshape(-1, 2), k=query_size)
                surroundings = ind[0][1:]
                for pos in surroundings:
                    if pos in visited:
                        continue
                    elif is_connected(cur, pos):
                        trim_corner(cur, pos)
                        if pos not in q:
                            q.append(pos)
                        if pos in edge:
                            edge.remove(pos)
                    else:
                        if pos not in q and pos not in edge: 
                            edge.append(pos)
            if len(edge) != 0:
                start = edge.popleft()
            else: 
                if len(unvisited) != 0: 
                    start = unvisited.pop()
                    unvisited.add(start)
                else: break
        return walls

        