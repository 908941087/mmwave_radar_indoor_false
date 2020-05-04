from sklearn.cluster import DBSCAN
import numpy as np
from PointCloudOperator import MorphologyOperator, PCBasics
from Environment import Environment
from EnvClassifier import EnvClassifier
from Cluster import Cluster
from shapely.geometry import MultiPoint


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

    def getEnv(self):
        if self.env is None:
            self.env = self.env_classifier.classify(self.clusters)
        return self.env

    def getEnhancedEnv(self):
        return self.enhanced_env


if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(1, 1, 1)
    ax.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')

    gp = GroupingTracker()
    gp.pc_group(PCBasics.getPCFromPCD("ti_ws/src/py_interface/scripts/EnvClassifier/pcds/3d_pc_map.pcd"))
    env = gp.getEnv()
    env.show(plt)
    env.showEntityTags(plt)

    plt.show()