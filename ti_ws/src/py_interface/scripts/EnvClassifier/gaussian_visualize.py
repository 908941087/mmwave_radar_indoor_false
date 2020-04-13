# coding=utf-8

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from utils import get_intersection, get_xy_lim
from regression_control import SimpleRegressionController, RegressionController
from points_divider import PointsDivider
from linear_regressor import *
from utils import get_slope

"""
TODO:
1. Maybe rewrite PointsDivider
2. Put outliers inside boxes
"""

verbose = True

# Linear regression
controller = RegressionController(path='four_walls.pcd', verbose=verbose)
divider = PointsDivider()
controller.set_parts(divider)
gau = GaussianRegressor()
controller.fit(gau)

# set figure

padding = 0.2
xy_lim = get_xy_lim(controller.points)
ratio = (xy_lim[3] - xy_lim[2]) / float(xy_lim[1] - xy_lim[0])
fig = plt.figure(figsize=(10 * ratio, 10))
fig.patch.set_facecolor('#000000')
ax1 = fig.add_subplot(1, 1, 1)
ax1.grid(True, linewidth=0.5, color='#999999', linestyle='dotted')
ax1.set_facecolor('#000000')
ax1.axis([xy_lim[0] - padding, xy_lim[1] + padding, xy_lim[2] - padding, xy_lim[3] + padding])

# scatter points
xy = controller.points
print("xy:",len(xy))
xs = [i[0] for i in xy]
ys = [i[1] for i in xy]
#print(type(xs))
#print(xs)
ax1.scatter(xs, ys, color='red', s=1)

#plot gaussian picture
parts=controller.parts
param=controller.parameters
#print(param)
#print(len(param))
#print(parts)

i=0
count=0
def func3(x, a1,  m1,  s1, ):
    return a1 * np.exp(-((x - m1) / s1) ** 2)
    #print(len(part.points))


for part in parts:
    #print(part.points)
    print(get_slope(part.points))
    print(len(part.points))
    count+=1
    if (0<get_slope(part.points) and get_slope(part.points)<45) or (get_slope(part.points) >135 and get_slope(part.points) <180) :
        #print(part.points)
        #print(get_slope(part.points))
        #print("########################")
        xy_lim=get_xy_lim(part.points)
        #print(part.points)
        #print(xy_lim[0])
        #print("####################")
        #print(len(part.points))
        #print("#####xy_lim",xy_lim)
        X_=np.arange(xy_lim[0],xy_lim[1],0.1)
        if(i==7):
            print("eee:",xy_lim[0],xy_lim[1])
            print("rrr:",param[i][0],param[i][1],param[i][2])
            
        yvals=gau.gaussian_mix(X_,param[i][0],param[i][1],param[i][2])
        plt.plot(X_,yvals,'r')  
        #print("0<90",i)            #break    
    else: 
        xy_lim=get_xy_lim(part.points)
        Y_=np.arange(xy_lim[2],xy_lim[3],0.1)
        xvals=gau.gaussian_mix(Y_,param[i][0],param[i][1],param[i][2])
        plt.plot(xvals,Y_,'r')
        #print("90<180",i)
        #print("90<180",len)
        #print(part.points)
        #break
    i+=1           
        
print(count)
# calculate intersections to plot
'''
intersections = controller.get_intersections()
for points in intersections:
    plt.plot([i[0] for i in points], [i[1] for i in points], color='pink', linewidth=15)
'''
# formatting the plot
# ax1.text(-0.9, 3.85, 'Variance: {:8.4f}'.format(variance(regressor.points, regressor.parameters, regressor.segments)), fontsize=10)

# ax1.set_title("Segmented Linear Regression(segments = {})".format(segments))
# plt.savefig("Segemented_Linear_Regression_segments_{}.png".format(str(segments)), dpi=300)
plt.show()