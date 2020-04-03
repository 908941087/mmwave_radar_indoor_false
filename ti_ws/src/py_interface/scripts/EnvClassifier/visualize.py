# coding=utf-8

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from utils import get_intersection, get_xy_lim, get_rectangle
from regression_control import SimpleRegressionController, RegressionController
from points_divider import PointsDivider
from linear_regressor import *

"""
TODO:
1. Maybe rewrite PointsDivider
2. Put outliers inside boxes
"""

verbose = True

# Linear regression
points = get_points_from_pcd('ti_ws/src/py_interface/scripts/EnvClassifier/south_one.pcd')
controller = RegressionController(verbose=verbose)
controller.set_points(points)
divider = PointsDivider()
controller.set_parts(divider, 0.5)
reg = LinearRegressor()
controller.fit(reg)

# set figure
padding = 0.2
xy_lim = get_xy_lim(controller.points)
ratio = (xy_lim[3] - xy_lim[2]) / float(xy_lim[1] - xy_lim[0])
fig = plt.figure(figsize=(10 * ratio, 10))
ax1 = fig.add_subplot(1, 1, 1)
ax1.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')
ax1.axis([xy_lim[0] - padding, xy_lim[1] + padding, xy_lim[2] - padding, xy_lim[3] + padding])

# scatter points
xy = controller.points
xs = [i[0] for i in xy]
ys = [i[1] for i in xy]
ax1.scatter(xs, ys, color='red', s=1)

# calculate intersections to plot
intersections = controller.get_intersections()
for points in intersections:
    if points is not None: plt.plot([i[0] for i in points], [i[1] for i in points], color='black', linewidth=3)

# draw rectangle for isolated parts
rects = [get_rectangle(p.points) for p in controller.parts if p.isolated]
for rect in rects:
    for edge in rect:
        plt.plot([i[0] for i in edge], [i[1] for i in edge], color='black', linewidth=3)


# formatting the plot
# ax1.text(-0.9, 3.85, 'Variance: {:8.4f}'.format(variance(regressor.points, regressor.parameters, regressor.segments)), fontsize=10)

# ax1.set_title("Segmented Linear Regression(segments = {})".format(segments))
# plt.savefig("Segemented_Linear_Regression_segments_{}.png".format(str(segments)), dpi=300)
plt.show()