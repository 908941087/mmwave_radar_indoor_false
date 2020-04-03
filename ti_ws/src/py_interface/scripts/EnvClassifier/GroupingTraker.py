from sklearn.cluster import DBSCAN
from points_generator import PointsGenerator
from ClassMarker import ClassMarker, Mark
from utils import *


class GroupingTracker:
    def __init__(self):
        self.pc2 = None
        self.db = None
        self.clusters_num = None
        self.clusters = []
        self.point_generator = PointsGenerator()
        self.class_marker = ClassMarker()

    def pc_group(self, pc2):
        self.pc2 = np.array(pc2)
        self.db = DBSCAN(eps=0.3, min_samples=10).fit(self.pc2)
        labels = self.db.labels_
        self.clusters_num = len(set(labels)) - (1 if -1 in labels else 0)
        for i in range(self.clusters_num):
            one_cluster = self.pc2[labels == i]
            self.clusters.append(one_cluster)
        return self.clusters

    def generate_points_per_cluster(self):
        res_points = []
        # TODO: Use ClassMarker to filter noise cluster and set marks
        self.class_marker.JudgeClass(self.clusters)
        for i in range(self.clusters_num):
            res_points.extend(self.point_generator.generate(self.clusters[i]))
        return res_points

    def show_clusters(self):
        labels = self.db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        for i in range(n_clusters_):
            one_cluster = self.pc2[labels == i]
            plt.plot(one_cluster[:, 0], one_cluster[:, 1], 'o')
        plt.show()

    def show_generated_points(self):
        pass


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    gt = GroupingTracker()
    source_points = get_points_from_pcd("ti_ws/src/py_interface/scripts/EnvClassifier/0.pcd")
    gt.pc_group(source_points)
    points = gt.generate_points_per_mark()
    # plt.scatter([p[0] for p in points], [p[1] for p in points], c='red', s=1)

    # index = 0
    # print(gt.class_marker.markers)
    # for cluster in gt.clusters:
    #     mark = gt.class_marker.markers[index][0]
    #     if mark is Mark.NOISE:
    #         plt.scatter([p[0] for p in cluster], [p[1] for p in cluster], c='r', s=1)
    #     elif mark is Mark.WALL:
    #         plt.scatter([p[0] for p in cluster], [p[1] for p in cluster], c='b', s=1)
    #     else:
    #         plt.scatter([p[0] for p in cluster], [p[1] for p in cluster], c='g', s=1)
    #     index += 1

    plt.show()