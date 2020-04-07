from utils import get_area, get_density, get_center, get_xy_lim, dist
from enum import Enum
import rospy
from visualization_msgs.msg import Marker
from wall_finder import WallFinder
import numpy as np


class Mark(Enum):
    NOISE = 0
    WALL = 1
    FURNITURE = 2


class ClassMarker:
    AREA_THRESHOLD = 0.25  # in square meter, area smaller than this threshold will be considered as noise
    DENSITY_THRESHOLD = 120  # lowest points per square meter
    RATIO_THRESHOLD = 10  # longer edge over shorter edge
    MAX_WALL_WIDTH = 0.4
    MIN_WALL_LENGTH = 1.1
    NOISE_POINTS_COUNT_THRESHOLD = 40
    DENSITY_PER_SQUARE_METER_THRESHOLD = 20000

    def __init__(self):
        self.markers = None  # wall, noise, furniture
        self.pub_markers = []
        self._show_noise = False
        self.CLASS_TIP_STR = {
            Mark.NOISE: "Noise",
            Mark.WALL: "Wall",
            Mark.FURNITURE: "Fnt"
        }

    def JudgeClass(self, clusters):
        if clusters is not None:
            self.markers = []
        else:
            return
        for cluster in clusters:
            # use aera and density to recognize noise
            area = get_area(cluster)
            density = len(cluster) / area
            if (area < self.AREA_THRESHOLD or density < self.DENSITY_THRESHOLD) and \
                len(cluster) < self.NOISE_POINTS_COUNT_THRESHOLD and density / area < self.DENSITY_PER_SQUARE_METER_THRESHOLD:
                self.markers.append({"mark": Mark.NOISE, "center": get_center(cluster), "area": area, "density": density})
                continue
            # try to treat this cluster as wall, see if it fits well
            wall_finder = WallFinder()
            walls = wall_finder.find_walls(cluster)
            avg_width = np.average([w["width"] for w in walls])
            total_length = sum([dist(ends[0], ends[1]) for ends in [w["ends"] for w in walls]])
            if avg_width > self.MAX_WALL_WIDTH or total_length < self.MIN_WALL_LENGTH or \
                total_length / avg_width < self.RATIO_THRESHOLD:
                self.markers.append({"mark": Mark.FURNITURE, "center": get_center(cluster), "area": area, "density": density})
                continue
            else: 
                self.markers.append({"mark": Mark.WALL, "center": get_center(cluster), "area": area, "density": density, "length": total_length, "width": avg_width, "walls": walls})


    def generate_markers(self, duration=5.0):
        mark_index = 0
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
            t_marker.text = self.CLASS_TIP_STR[marker_info[0]]
            # Size
            t_marker.scale.x = 0.2
            t_marker.scale.y = 0.2
            t_marker.scale.z = 0.2

            # ADD/DELETE
            t_marker.action = Marker.ADD

            # Pose
            t_marker.pose.position.x = marker_info[1][0]
            t_marker.pose.position.y = marker_info[1][1]
            t_marker.pose.position.z = 0.2
            t_marker.pose.orientation.x = 0.0
            t_marker.pose.orientation.y = 0.0
            t_marker.pose.orientation.z = 0.0
            t_marker.pose.orientation.w = 1.0

            # Color
            t_marker.color.r = 1.0
            t_marker.color.g = 0.95
            t_marker.color.b = 0.8
            t_marker.color.a = 0.7

            t_marker.lifetime = rospy.Duration()
            self.pub_markers.append(t_marker)
        return self.pub_markers
