from enum import Enum
import numpy as np
from sklearn.neighbors import KDTree
from Environment import Environment
from Entity import Wall, Furniture, Door, Noise
from PointCloudOperator import ClusterFit

class EnvClassifier(object):
    AREA_THRESHOLD = 0.25  # in square meter, area smaller than this threshold will be considered as noise
    DENSITY_THRESHOLD = 120  # lowest points per square meter
    RATIO_THRESHOLD = 10  # longer edge over shorter edge
    MAX_WALL_WIDTH = 0.4
    MIN_WALL_LENGTH = 1.1
    NOISE_POINTS_COUNT_THRESHOLD = 40
    DENSITY_PER_SQUARE_METER_THRESHOLD = 20000
    MIN_ISOLATION_DIST = 0.7 # meters

    def __init__(self):
        self.distances = {} # cluster to cluster distance
        self._show_noise = False
        
    def classify(self, clusters):
        if clusters is None:
            return
        env = Environment()
        self.distances = {}

        cluster_centers = np.array([c.getCenter() for c in clusters]).reshape(-1, 2)
        
        # mark clusters for the first round
        for i in range(len(clusters)):
            cluster = clusters[i]

            # use aera and density to recognize noise
            area = cluster.getArea()
            if area > 0:
                density = cluster.getDensity()
            if (area < self.AREA_THRESHOLD or density < self.DENSITY_THRESHOLD) and \
                    cluster.getPointsCount() < self.NOISE_POINTS_COUNT_THRESHOLD and density / area < self.DENSITY_PER_SQUARE_METER_THRESHOLD:
                env.register(Noise(cluster.getId()), cluster)
                continue

            # try to treat this cluster as wall, see if it fits well
            wall = ClusterFit.wallFit(cluster)
            if wall.getWidth() > self.MAX_WALL_WIDTH or wall.getLength() < self.MIN_WALL_LENGTH or \
                    wall.getLength() / wall.getWidth() < self.RATIO_THRESHOLD:
                env.register(Furniture(cluster.getId(), cluster.getConvexHull()), cluster)
                continue
            else:
                env.register(wall, cluster)

        # build KDTree using cluster centers that don't contain noise
        # clusters_without_noise = [clusters[i] for i in range(len(clusters)) if self.markers[i]["mark"] is not Mark.NOISE]
        # cluster_centers_without_noise = np.array([cluster_centers[i] for i in range(len(cluster_centers)) if self.markers[i]["mark"] is not Mark.NOISE])
        # if len(cluster_centers_without_noise) in [0, 1]: return
        
        # tree = KDTree(cluster_centers_without_noise, leaf_size=2)
        # query_size = 6
        # if query_size > len(cluster_centers_without_noise): query_size = len(cluster_centers_without_noise)

        # for i in range(len(cluster_centers)):
        #     # use KDTree to find the nearest cluster, check if current cluster is isolated
        #     distance, ind = tree.query(cluster_centers[i].reshape(-1, 2), k=query_size)
        #     min_dist = min([self.dist_cluster2cluster(clusters[i], clusters_without_noise[index], list(cluster_centers[i]), list(cluster_centers_without_noise[index])) for index in ind[0][1:]])
        #     self.markers[i]["isolated"] = min_dist > self.MIN_ISOLATION_DIST
        #     self.markers[i]["min_dist"] = min_dist

        # # remark walls that are isolated as obstacles
        # for marker, cluster in zip(self.markers, clusters):
        #     if marker["isolated"] and marker["mark"] is Mark.WALL:
        #         marker["mark"] = Mark.OBSTACLE
        #         del marker["length"]
        #         del marker["width"]
        #         del marker["walls"]
        return env