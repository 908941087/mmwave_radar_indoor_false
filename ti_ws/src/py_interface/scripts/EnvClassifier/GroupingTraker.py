from sklearn.cluster import DBSCAN
import numpy as np
from PointCloudOperator import MorphologyOperator, PCBasics
from Environment import Environment
from EnvClassifier import EnvClassifier
from Cluster import Cluster, ClusterType
from shapely.geometry import MultiPoint, Point


class GroupingTracker:
    def __init__(self):
        self.env_classifier = EnvClassifier()
        self.env = None
        self.enhanced_env = None

    def pc_group(self, laser_pc, mmwave_pc):
        laser_clusters = []
        mmwave_clusters = []
        laser_pc = np.array(laser_pc)
        mmwave_pc = np.array(mmwave_pc)

        # using DBSCAN to fit laser point cloud(2D)
        db = DBSCAN(eps=0.3, min_samples=10).fit(laser_pc)
        labels = db.labels_
        clusters_num = len(set(labels)) - (1 if -1 in labels else 0)
        for i in range(clusters_num):
            one_cluster = laser_pc[labels == i]
            laser_clusters.append(Cluster(i, ClusterType.LASER, 2, MultiPoint(one_cluster)))

        # using DBSCAN to fit mmwave point cloud(3D)
        db = DBSCAN(eps=0.3, min_samples=10).fit(mmwave_pc)
        labels = db.labels_
        clusters_num = len(set(labels)) - (1 if -1 in labels else 0)
        for i in range(clusters_num):
            one_cluster = mmwave_pc[labels == i]
            mmwave_clusters.append(Cluster(i, ClusterType.MMWAVE, 3, MultiPoint(one_cluster)))
        return laser_clusters, mmwave_clusters

    def getEnv(self, mmwave_pc, laser_grid):
        width, height = laser_grid.info.width, laser_grid.info.height
        x_offset, y_offset = laser_grid.info.origin.position.x, laser_grid.info.origin.position.y
        resolution = laser_grid.info.resolution
        laser_array = np.array(laser_grid.data).reshape(height, width)
        laser_pc = [[j * resolution + x_offset, i * resolution + y_offset] for i in range(height) for j in range(width)
                    if laser_array[i][j] > 0]
        if len(laser_pc) == 0:
            return Environment()
        laser_clusters, mmwave_clusters = self.pc_group(laser_pc, mmwave_pc)
        self.env = self.env_classifier.classify(laser_clusters, mmwave_clusters)
        # import matplotlib
        # matplotlib.use('TkAgg')
        # import matplotlib.pyplot as plt
        # fig = plt.figure(figsize=(10, 10))
        # ax = fig.add_subplot(1, 1, 1)
        # ax.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')
        # cs = ['r', 'g', 'b', 'pink', 'cyan', 'orange']
        # index = 0
        # for a_cluster in self.clusters:
        #     points = a_cluster.getPoints()
        #     plt.scatter([p.x for p in points], [p.y for p in points], s=20, c=cs[index % len(cs)], edgecolors='none')
        #     index += 1
        # # plt.scatter([p[0] for p in laser_pc], [p[1] for p in laser_pc], s=1, c='r')
        # self.env.showClusters(plt)
        # self.env.showEntityShapes(plt)
        # self.env.showEntityTags(plt)
        # plt.show()
        return self.env

    def getEnhancedEnv(self, pc2):
        self.enhanced_env = self.getEnv(pc2).enhance()
        return self.enhanced_env


if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(1, 1, 1)
    ax.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')

    gp = GroupingTracker()
    env = gp.getEnhancedEnv(PCBasics.getPCFromPCD("pcds/south_one.pcd"))
    env.show(plt)
    env.showEntityShapes(plt)
    # env.showEntityTags(plt)

    plt.show()