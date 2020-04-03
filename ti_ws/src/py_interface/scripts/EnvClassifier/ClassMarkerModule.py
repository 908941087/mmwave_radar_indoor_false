from utils import get_area, get_density, get_center, get_xy_lim
from enum import Enum
import rospy
from visualization_msgs.msg import Marker


class Mark(Enum):
    NOISE = 0
    WALL = 1
    FURNITURE = 2


class ClassMarker:
    AREA_THRESHOLD = 0.5  # square meter
    DENSITY_THRESHOLD = 20  # lowest points per square meter
    RATIO_THRESHOLD = 2  # longer edge over shorter edge

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
            area = get_area(cluster)
            if area < self.AREA_THRESHOLD:
                self.markers.append([Mark.NOISE, get_center(cluster)])
                continue
            density = get_density(cluster)
            if density < self.DENSITY_THRESHOLD:
                self.markers.append([Mark.NOISE, get_center(cluster)])
                continue
            xy_lim = get_xy_lim(cluster)
            ratio = (xy_lim[3] - xy_lim[2]) / (xy_lim[1] - xy_lim[0])
            if ratio < 1: ratio = 1 / ratio
            if ratio < self.RATIO_THRESHOLD:
                self.markers.append([Mark.FURNITURE, get_center(cluster)])
                continue
            self.markers.append([Mark.WALL, get_center(cluster)])

    def generate_markers(self):
        mark_index = 0
        for marker_info in self.markers:
            if not self._show_noise and marker_info[0] == Mark.NOISE:
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
