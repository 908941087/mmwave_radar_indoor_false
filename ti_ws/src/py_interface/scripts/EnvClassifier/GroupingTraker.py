from sklearn.cluster import DBSCAN
import numpy as np
from PointCloudOperator import MorphologyOperator, PCBasics
from Environment import Environment
from EnvClassifier import EnvClassifier
from Cluster import Cluster, ClusterType
from shapely.geometry import MultiPoint, Point
from timer import timer
from datetime import datetime

import rospy


class GroupingTracker:
    def __init__(self):
        self.env_classifier = EnvClassifier()
        self.env = None
        self.enhanced_env = None
        self.last_laser_occupancy_count = 0
        self.laser_clusters = None
        self.mmwave_clusters = None
        self.last_update_time = datetime.now()

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
            mmwave_clusters.append(Cluster(i, ClusterType.MMWAVE, 3, MultiPoint(list(one_cluster))))
        return laser_clusters, mmwave_clusters

    @timer
    def getEnv(self, mmwave_pc, laser_grid):
        # convert laser grid to laser pc
        width, height = laser_grid.info.width, laser_grid.info.height
        x_offset, y_offset = laser_grid.info.origin.position.x, laser_grid.info.origin.position.y
        resolution = laser_grid.info.resolution
        laser_array = np.array(laser_grid.data).reshape(height, width)
        laser_pc = [[j * resolution + x_offset, i * resolution + y_offset] for i in range(height) for j in range(width)
                    if laser_array[i][j] > 0]
        if len(laser_pc) == 0:
            return Environment()
        cur_laser_occupancy_count = len(laser_pc)
        time_delta = datetime.now() - self.last_update_time
        if (self.last_laser_occupancy_count == 0 or
                cur_laser_occupancy_count / float(self.last_laser_occupancy_count) > 1.3 or
                (time_delta.seconds > 90
                 and cur_laser_occupancy_count / float(self.last_laser_occupancy_count) > 1.1) or
                time_delta.seconds > 120):
            laser_clusters, mmwave_clusters = self.pc_group(laser_pc, mmwave_pc)
            self.env = self.env_classifier.classify(laser_clusters, mmwave_clusters)
            self.laser_clusters = laser_clusters
            self.mmwave_clusters = mmwave_clusters
            self.last_laser_occupancy_count = cur_laser_occupancy_count
            self.last_update_time = datetime.now()
            rospy.logwarn("Update condition satisfied. Updating Env.")
        else:
            rospy.logwarn("Update condition not satisfied. Using Old Env.")
        return self.env, self.laser_clusters, self.mmwave_clusters

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