from sklearn.cluster import DBSCAN
from points_generator import PointsGenerator
from ClassMarkerModule import ClassMarker, Mark
from wall_finder import WallFinder
from utils import *


class GroupingTracker:
    def __init__(self):
        self.pc2 = None
        self.db = None
        self.clusters_num = None
        self.clusters = []
        self.point_generator = PointsGenerator()
        self.class_marker = ClassMarker()
        self.wall_finder = WallFinder()


    def pc_group(self, pc2):
        self.pc2 = np.array(pc2)
        self.db = DBSCAN(eps=0.3, min_samples=10).fit(self.pc2)
        labels = self.db.labels_
        self.clusters_num = len(set(labels)) - (1 if -1 in labels else 0)
        for i in range(self.clusters_num):
            one_cluster = self.pc2[labels == i]
            self.clusters.append(one_cluster)
        self.class_marker.JudgeClass(self.clusters)
        return self.clusters

    def generate_points_per_cluster(self, pc2):
        self.pc_group(pc2)
        res_points = []
        self.class_marker.JudgeClass(self.clusters)
        for i in range(self.clusters_num):
            # res_points.append(self.point_generator.generate(self.clusters[i]))
            if(self.class_marker.markers[i][0] != Mark.NOISE):
                res_points.append([p[0], p[1]] for p in self.clusters[i])
        return res_points

    def generate_points_per_mark(self, pc2):
        self.pc_group(pc2)
        res_points = []
        self.class_marker.JudgeClass(self.clusters)
        clusters = {mark : [] for mark in Mark}
        for i in range(len(self.clusters)):
            mark = self.class_marker.markers[i][0]
            cluster = self.clusters[i]
            clusters[mark].extend(cluster)
        for mark in Mark:
            if mark is Mark.WALL:
                # plt.scatter([p[0] for p in clusters[mark]], [p[1] for p in clusters[mark]], c='r', s=1)
                walls = self.wall_finder.find_walls(clusters[mark])
                for w in walls:
                #     plt.plot([i[0] for i in w['ends']], [i[1] for i in w['ends']], c='b', linewidth=2)
                    res_points.extend(self.point_generator.generate_for_line(w['line'], w['ends'], w['width']))
        plt.scatter([p[0] for p in res_points], [p[1] for p in res_points], c='b', s=1)
        return res_points

    def generate_makers(self):
        return self.class_marker.generate_markers()

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
    points = gt.generate_points_per_mark(source_points)
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