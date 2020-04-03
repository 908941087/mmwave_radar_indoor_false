from utils import get_area, get_density, get_center, get_xy_lim


class ClassMarker:
    self.AREA_THRESHOLD = 0.5 # square meter
    self.DENSITY_THRESHOLD = 20  # lowest points per square meter
    self.RATIO_THRESHOLD = 2 # longer edge over shorter edge

    def __init__(self):
        self.markers = None # wall, noise, furniture

    def JudgeClass(self, clusters):
        if clusters is not None: 
            self.markers = []
        else: return 
        for cluster in clusters:
            area =  get_area(cluster)
            if area < self.AREA_THRESHOLD:
                self.markers.append(["noise", get_center(cluster)])
                continue
            density = get_density(cluster)
            if density < self.DENSITY_THRESHOLD:
                self.markers.append(["noise", get_center(cluster)])
                continue
            xy_lim =  get_xy_lim(cluster)
            ratio = (xy_lim[3] - xy_lim[2]) / (xy_lim[1] - xy_lim[0])
            if ratio < 1: ratio = 1 / ratio
            if ratio < self.RATIO_THRESHOLD:
                self.markers.append(["furniture", get_center(cluster)])
                continue
            self.markers.append(["wall", get_center(cluster)])

