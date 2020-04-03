from utils import get_area, get_density, get_center, get_xy_lim
from enum import Enum

class Mark(Enum):
    NOISE = 0
    WALL = 1
    FURNITURE = 2


class ClassMarker:
    AREA_THRESHOLD = 0.5 # square meter
    DENSITY_THRESHOLD = 20  # lowest points per square meter
    RATIO_THRESHOLD = 2 # longer edge over shorter edge
    

    def __init__(self):
        self.markers = None # wall, noise, furniture

    def JudgeClass(self, clusters):
        if clusters is not None: 
            self.markers = []
        else: return 
        for cluster in clusters:
            area =  get_area(cluster)
            if area < self.AREA_THRESHOLD:
                self.markers.append([Mark.NOISE, get_center(cluster)])
                continue
            density = get_density(cluster)
            if density < self.DENSITY_THRESHOLD:
                self.markers.append([Mark.NOISE, get_center(cluster)])
                continue
            xy_lim =  get_xy_lim(cluster)
            ratio = (xy_lim[3] - xy_lim[2]) / (xy_lim[1] - xy_lim[0])
            if ratio < 1: ratio = 1 / ratio
            if ratio < self.RATIO_THRESHOLD:
                self.markers.append([Mark.FURNITURE, get_center(cluster)])
                continue
            self.markers.append([Mark.WALL, get_center(cluster)])

