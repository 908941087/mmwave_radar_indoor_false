from shapely.ops import triangulate
from shapely.geometry import Point, Polygon
import random
import math

def triSample(triangle):
    """
    Given three vertices A, B, C,
    sample point uniformly in the triangle
    """
    coords = triangle.exterior.coords
    if not isinstance(triangle, Polygon) or len(coords) != 4:
        raise TypeError("Input must be a triangle!")
    A = coords[0]
    B = coords[1]
    C = coords[2]

    r1 = random.random()
    r2 = random.random()

    s1 = math.sqrt(r1)

    x = A[0] * (1.0 - s1) + B[0] * (1.0 - r2) * s1 + C[0] * r2 * s1
    y = A[1] * (1.0 - s1) + B[1] * (1.0 - r2) * s1 + C[1] * r2 * s1

    return Point(x, y)


def generateForTriangle(triangle, intensity):
    points_count = int(triangle.area * intensity)
    return [triSample(triangle) for _ in range(points_count)]


def generateForPolygon(poly, x_offset=0.05, y_offset=0.05):
    min_x, min_y, max_x, max_y = poly.bounds
    col = int(math.ceil((max_x - min_x) / x_offset))
    row = int(math.ceil((max_y - min_y) / y_offset))
    res_points = []
    for i in range(col):
        for j in range(row):
            p = Point(x_offset * i + min_x, y_offset * j + min_y)
            if poly.contains(p):
                res_points.append(p)
    return res_points
