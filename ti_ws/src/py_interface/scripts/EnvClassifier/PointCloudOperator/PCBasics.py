import pypcd
import numpy as np

def getPCFromPCD(path):
    return [[i['x'], i['y']] for i in pypcd.PointCloud.from_path(path).pc_data]

def filterWithRect(points, x_low=float('-inf'), x_high=float('inf'), y_low=float('-inf'), y_high=float('inf')):
    result = [i for i in points if x_low <= i[0] < x_high and y_low <= i[1] < y_high]
    return result

def getXYLim(points):
    x = [i[0] for i in points]
    y = [i[1] for i in points]
    return [min(x), max(x), min(y), max(y)]

def fitLine(points):
    """
    并非一元线性回归，详见
    https://blog.csdn.net/liyuanbhu/article/details/50866802?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task
    """
    count = len(points)
    if (count < 2): return 0, 0, 0
    x_mean = sum([p[0] for p in points]) / float(count)
    y_mean = sum([p[1] for p in points]) / float(count)

    Dxx = Dxy = Dyy = 0.0
    for i in range(count):
        Dxx += pow((points[i][0] - x_mean), 2)
        Dxy += (points[i][0] - x_mean) * (points[i][1] - y_mean)
        Dyy += pow((points[i][1] - y_mean), 2)

    Lambda = ((Dxx + Dyy) - np.sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0
    den = np.sqrt(Dxy * Dxy + (Lambda - Dxx) * (Lambda - Dxx))

    a = Dxy / den
    b = (Lambda - Dxx) / den
    c = - a * x_mean - b * y_mean
    return a, b, c

def getKB(points):
    a, b, c = fitLine(points)
    if b != 0:
        return - a / b, - c / b
    elif a != 0:
        return float('inf'), - c / a
    else:
        return float('inf'), float('inf')