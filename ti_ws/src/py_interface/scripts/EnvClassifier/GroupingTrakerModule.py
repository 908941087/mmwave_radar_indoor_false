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

    def pc_group(self, pc2):
        self.clusters = []
        self.pc2 = np.array(pc2)
        self.db = DBSCAN(eps=0.3, min_samples=10).fit(self.pc2)
        labels = self.db.labels_
        self.clusters_num = len(set(labels)) - (1 if -1 in labels else 0)
        for i in range(self.clusters_num):
            one_cluster = self.pc2[labels == i]
            self.clusters.append(one_cluster)
        self.class_marker.JudgeClass(self.clusters)
        return self.clusters

    def generate_points_per_cluster(self, pc2, linear_regression=False):
        self.pc_group(pc2)
        res_points = []
        for i in range(self.clusters_num):
            # res_points.append(self.point_generator.generate(self.clusters[i]))
            if self.class_marker.markers[i]["mark"] is Mark.WALL:
                wall_finder = WallFinder()
                cluster = self.clusters[i]
                walls = wall_finder.find_walls(cluster)
                for w in walls:
                    if linear_regression:
                        res_points.extend(self.point_generator.generate_for_line(w['line'], w['ends'], w['width']))
                    else:
                        res_points.extend(self.clusters[i])
            if self.class_marker.markers[i]["mark"] is Mark.FURNITURE:
                res_points.extend(self.clusters[i])
        return res_points

    def generate_points_per_mark(self, pc2):
        self.pc_group(pc2)
        res_points = []
        clusters = {mark: [] for mark in Mark}
        for i in range(len(self.clusters)):
            mark = self.class_marker.markers[i]["mark"]
            cluster = self.clusters[i]
            clusters[mark].extend(cluster)
        for mark in Mark:
            if mark is Mark.WALL:
                wall_finder = WallFinder()
                walls = wall_finder.find_walls(clusters[mark])
                plt.scatter([p[0] for p in clusters[mark]], [p[1] for p in clusters[mark]], c='r', s=1)
                if walls is not None:
                    for w in walls:
                        plt.plot([i[0] for i in w['ends']], [i[1] for i in w['ends']], c='b', linewidth=2)
                        res_points.extend(self.point_generator.generate_for_line(w['line'], w['ends'], w['width']))
            if mark is Mark.FURNITURE:
                res_points.extend(clusters[mark])
        return res_points

    def generate_makers(self):
        return self.class_marker.generate_markers()

    def show_clusters(self):
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        labels = self.db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        for i in range(n_clusters_):
            one_cluster = self.pc2[labels == i]
            plt.plot(one_cluster[:, 0], one_cluster[:, 1], 'o')
        plt.show()

    def show_generated_points(self, source_points):
        ax = self.plt_setup()
        generated_points = self.generate_points_per_cluster(source_points)
        ax.scatter([p[0] for p in source_points], [p[1] for p in source_points], c='r', s=1)
        ax.scatter([p[0] for p in generated_points], [p[1] for p in generated_points], c='b', s=1)
        plt.show()

    def plt_setup(self):
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(10, 10))
        ax1 = fig.add_subplot(1, 1, 1)
        ax1.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')
        return ax1

    def show_marked_clusters(self, source_points, verbose=False):
        self.pc_group(source_points)
        ax = self.plt_setup()

        index = 0
        for cluster in self.clusters:
            marker = self.class_marker.markers[index]

            information = ""
            if verbose:
                information = "\narea: " + str(round(marker["area"], 2)) + \
                              "\ndensity: " + str(round(marker["density"], 2)) + \
                              "\npoint count: " + str(len(cluster)) + \
                              "\n d / a: " + str(round(marker["density"] / marker["area"], 2))

            if marker["mark"] is Mark.NOISE:
                ax.scatter([p[0] for p in cluster], [p[1] for p in cluster], c='r', s=1)
                information = "NOISE" + information
            elif marker["mark"] is Mark.WALL:
                ax.scatter([p[0] for p in cluster], [p[1] for p in cluster], c='b', s=1)
                information = "WALL" + information
                if verbose:
                    information += "\nratio: " + str(round(marker["length"] / marker["width"], 2)) + \
                                   "\nlength: " + str(round(marker["length"], 2)) + \
                                   "\nwidth: " + str(round(marker["width"], 2))
                # walls = marker["walls"]
                # for w in walls:
                #     plt.plot([i[0] for i in w['ends']], [i[1] for i in w['ends']], c='b', linewidth=2)
            else:
                ax.scatter([p[0] for p in cluster], [p[1] for p in cluster], c='g', s=1)
                information = "FURNITURE" + information
            ax.text(marker["center"][0], marker["center"][1], information, style='italic', fontsize=6,
                    bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 5})
            index += 1

        plt.show()


if __name__ == '__main__':
    gt = GroupingTracker()
    source_points = get_points_from_pcd("ti_ws/src/py_interface/scripts/EnvClassifier/pcds/0.pcd")
    # gt.show_generated_points(source_points)
    gt.show_marked_clusters(source_points, verbose=False)
    # gt.show_clusters()
    # points = gt.generate_points_per_cluster(source_points)
