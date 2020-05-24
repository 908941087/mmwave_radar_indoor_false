from sklearn.cluster import DBSCAN
import numpy as np
from PointCloudOperator import MorphologyOperator, PCBasics
from Environment import Environment
from EnvClassifier import EnvClassifier
from Cluster import Cluster
from shapely.geometry import MultiPoint, Point


class GroupingTracker:
    def __init__(self):
        self.pc2 = None
        self.db = None
        self.clusters_num = None
        self.clusters = []
        self.env_classifier = EnvClassifier()
        self.env = None
        self.enhanced_env = None

    def pc_group(self, pc2):
        self.clusters = []
        self.pc2 = np.array(pc2)
        self.db = DBSCAN(eps=0.3, min_samples=10).fit(self.pc2)
        labels = self.db.labels_
        self.clusters_num = len(set(labels)) - (1 if -1 in labels else 0)
        for i in range(self.clusters_num):
            one_cluster = self.pc2[labels == i]
            self.clusters.append(Cluster(i, MultiPoint(one_cluster)))
        return self.clusters

    def getEnv(self, mmwave_pc2, laser_grid):
        width, height = laser_grid.info.width, laser_grid.info.height
        x_offset, y_offset = laser_grid.info.origin.position.x, laser_grid.info.origin.position.y
        resolution = laser_grid.info.resolution
        laser_array = np.array(laser_grid.data).reshape(height, width)
        laser_pc = [[j * resolution + y_offset, i * resolution + x_offset] for i in range(height) for j in range(width)
                    if laser_array[i][j] > 0]
        if len(laser_pc) == 0:
            return Environment()
        self.pc_group(laser_pc)
        self.env = self.env_classifier.classify(self.clusters)
        # addEnvHeight(self.env, mmwave_pc2)  # TODO: Implement this.
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')
        cs = ['r', 'g', 'b', 'pink', 'cyan', 'orange']
        index = 0
        for a_cluster in self.clusters:
            points = a_cluster.getPoints()
            plt.scatter([p.x for p in points], [p.y for p in points], s=20, c=cs[index % len(cs)], edgecolors='none')
            index += 1
        # # plt.scatter([p[0] for p in laser_pc], [p[1] for p in laser_pc], s=1, c='r')
        # self.env.showClusters(plt)
        # self.env.showEntityShapes(plt)
        # self.env.showEntityTags(plt)
        plt.show()
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