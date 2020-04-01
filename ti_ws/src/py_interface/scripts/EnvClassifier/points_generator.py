from points_divider import PointsDivider
from linear_regressor import LinearRegressor
from regression_control import RegressionController
from utils import *

class PointsGenerator:

    def __init__(self, intensity=20):
        """
        @param intensity: how many points should we have given the length of 1 meter
        """
        self.intensity = float(intensity)

    def generate(self, points):
        divider = PointsDivider()
        controller = RegressionController()
        controller.set_points(points)
        controller.set_parts(divider, 0.5)
        reg = LinearRegressor()
        controller.fit(reg)
        result = []
        
        for part, param, ends in zip(controller.parts, controller.parameters, controller.intersections):
            if not part.isolated: 
                result.extend(self.generate_for_line(param[0], ends, self.get_linewidth(part.points, param[0])))
        return result

    def generate_for_line(self, line, ends, linewidth):
        """
        @param line: the line to generate points, [k, b], for y = k * x + b
        @param ends: two ends of the line
        @param linewidth: the width of the line
        @return: points generated to mimic the line
        """
        length = dist(ends[0], ends[1])
        row = int(round(linewidth * self.intensity))
        col = int(round(length * self.intensity))
        points = [[i / self.intensity + linewidth / 2.0, j / self.intensity] for i in range(col) for j in range(row)]
        points = rotate_points(points, k2slope(line[0]))
        lower_end = []
        if ends[0][1] < ends[1][1]:
            lower_end = ends[0]
        else: lower_end = ends[1]
        points = translate_points(points, [-lower_end[0], -lower_end[1]])
        return points

    @staticmethod
    def get_linewidth(points, line):
        a = line[0]
        b = -1
        c = line[1]
        temp = sum([abs(a * p[0] + b * p[1] + c) for p in points])
        return temp / np.sqrt(a ** 2 + 1) / len(points) * 2.0


if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    generator = PointsGenerator(30)
    source_points = get_points_from_pcd('0.pcd')
    points = generator.generate(source_points)

    padding = 0.2
    xy_lim = get_xy_lim(source_points)
    ratio = (xy_lim[3] - xy_lim[2]) / float(xy_lim[1] - xy_lim[0])
    fig = plt.figure(figsize=(10 * ratio, 10))
    fig.patch.set_facecolor('#000000')
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')
    ax1.set_facecolor('#000000')
    ax1.axis([xy_lim[0] - padding, xy_lim[1] + padding, xy_lim[2] - padding, xy_lim[3] + padding])

    ax1.scatter([p[0] for p in points], [p[1] for p in points], c='blue', s=1)
    ax1.scatter([0], [0], c='white', s=4)

    plt.show()