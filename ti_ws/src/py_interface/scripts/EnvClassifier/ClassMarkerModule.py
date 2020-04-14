from utils import get_area, get_density, get_center, get_xy_lim, dist
from enum import Enum
import rospy
from visualization_msgs.msg import Marker
from wall_finder import WallFinder
import numpy as np
from sklearn.neighbors import KDTree


class Mark(Enum):
    NOISE = 0
    WALL = 1
    FURNITURE = 2
    OBSTACLE = 3


class ClassMarker:
    AREA_THRESHOLD = 0.25  # in square meter, area smaller than this threshold will be considered as noise
    DENSITY_THRESHOLD = 120  # lowest points per square meter
    RATIO_THRESHOLD = 10  # longer edge over shorter edge
    MAX_WALL_WIDTH = 0.4
    MIN_WALL_LENGTH = 1.1
    NOISE_POINTS_COUNT_THRESHOLD = 40
    DENSITY_PER_SQUARE_METER_THRESHOLD = 20000
    MIN_ISOLATION_DIST = 0.7 # meters

    CLASS_TIP_STR = {
        Mark.NOISE: "Noise",
        Mark.WALL: "Wall",
        Mark.FURNITURE: "Fnt",
        Mark.OBSTACLE: "Obst"
    }

    def __init__(self):
        self.markers = None  # wall, noise, furniture
        self.distances = {}
        self.pub_markers = []
        self._show_noise = False
        
    def JudgeClass(self, clusters):
        if clusters is None:
            return
        self.markers = []
        self.distances = {}

        cluster_centers = np.array([get_center(c) for c in clusters]).reshape(-1, 2)
        
        # mark clusters for the first round
        for i in range(len(clusters)):
            cluster = clusters[i]
            info = {"ID": i, "center": cluster_centers[i]}

            # use aera and density to recognize noise
            info["area"] = get_area(cluster)
            info["density"] = len(cluster) / info["area"]
            if (info["area"] < self.AREA_THRESHOLD or info["density"] < self.DENSITY_THRESHOLD) and \
                    len(cluster) < self.NOISE_POINTS_COUNT_THRESHOLD and info["density"] / info["area"] < self.DENSITY_PER_SQUARE_METER_THRESHOLD:
                info["mark"] = Mark.NOISE
                self.markers.append(info)
                continue
            # try to treat this cluster as wall, see if it fits well
            wall_finder = WallFinder()
            walls = wall_finder.find_walls(cluster)
            avg_width = np.average([w["width"] for w in walls])
            total_length = sum([dist(ends[0], ends[1]) for ends in [w["ends"] for w in walls]])
            if avg_width > self.MAX_WALL_WIDTH or total_length < self.MIN_WALL_LENGTH or \
                    total_length / avg_width < self.RATIO_THRESHOLD:
                info["mark"] = Mark.OBSTACLE
                self.markers.append(info)
                continue
            else:
                info["mark"] = Mark.WALL
                info["length"] = total_length
                info["width"] = avg_width
                info["walls"] = walls
                self.markers.append(info)

        # build KDTree using cluster centers that don't contain noise
        clusters_without_noise = [clusters[i] for i in range(len(clusters)) if self.markers[i]["mark"] is not Mark.NOISE]
        cluster_centers_without_noise = np.array([cluster_centers[i] for i in range(len(cluster_centers)) if self.markers[i]["mark"] is not Mark.NOISE])
        if len(cluster_centers_without_noise) in [0, 1]: return
        
        tree = KDTree(cluster_centers_without_noise, leaf_size=2)
        query_size = 6
        if query_size > len(cluster_centers_without_noise): query_size = len(cluster_centers_without_noise)

        for i in range(len(cluster_centers)):
            # use KDTree to find the nearest cluster, check if current cluster is isolated
            distance, ind = tree.query(cluster_centers[i].reshape(-1, 2), k=query_size)
            min_dist = min([self.dist_cluster2cluster(clusters[i], clusters_without_noise[index], list(cluster_centers[i]), list(cluster_centers_without_noise[index])) for index in ind[0][1:]])
            self.markers[i]["isolated"] = min_dist > self.MIN_ISOLATION_DIST
            self.markers[i]["min_dist"] = min_dist

        # remark walls that are isolated as obstacles
        for marker, cluster in zip(self.markers, clusters):
            if marker["isolated"] and marker["mark"] is Mark.WALL:
                marker["mark"] = Mark.OBSTACLE
                del marker["length"]
                del marker["width"]
                del marker["walls"]

    def dist_cluster2cluster(self, cluster1, cluster2, center1, center2):
        key = ""
        if center1[0] > center2[0]:
            key = str(center2) + "," + str(center1)
        else:
            key = str(center1) + "," + str(center2)
        if self.distances.has_key(key): return self.distances[key]
        min_dist = float('inf')
        p2 = None
        for p in cluster2:
            temp = dist(p, center1)
            if min_dist > temp:
                min_dist = temp
                p2 = p
        min_dist = float('inf')
        for p in cluster1:
            temp = dist(p, p2)
            if min_dist > temp:
                min_dist = temp
        self.distances[key] = min_dist
        return min_dist

    def generate_markers(self, duration=5.0):
        mark_index = 0
        self.pub_markers = []
        for marker_info in self.markers:
            if not self._show_noise and marker_info["mark"] == Mark.NOISE:
                continue
            t_marker = Marker()
            t_marker.header.frame_id = "/map"
            t_marker.header.stamp = rospy.Time.now()
            t_marker.ns = "cluster_class"

            t_marker.id = mark_index
            mark_index += 1
            # Type
            t_marker.type = Marker.TEXT_VIEW_FACING
            t_marker.text = self.CLASS_TIP_STR[marker_info["mark"]]
            # Size
            t_marker.scale.x = 0.3
            t_marker.scale.y = 0.3
            t_marker.scale.z = 0.3

            # ADD/DELETE
            t_marker.action = Marker.MODIFY

            # Pose
            t_marker.pose.position.x = marker_info["center"][0]
            t_marker.pose.position.y = marker_info["center"][1]
            t_marker.pose.position.z = 0.2
            t_marker.pose.orientation.x = 0.0
            t_marker.pose.orientation.y = 0.0
            t_marker.pose.orientation.z = 0.0
            t_marker.pose.orientation.w = 1.0

            # Color
            t_marker.color.r = 0.0
            t_marker.color.g = 1.0
            t_marker.color.b = 0.5
            t_marker.color.a = 1.0

            t_marker.lifetime = rospy.Duration(duration)
            self.pub_markers.append(t_marker)
        return self.pub_markers
