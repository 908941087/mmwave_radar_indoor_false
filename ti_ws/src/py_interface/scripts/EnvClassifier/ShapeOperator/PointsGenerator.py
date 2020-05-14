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
    if not isinstance(triangle, Polygon) or len(coords) != 3:
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
    points_count = triangle.area * intensity
    return [triSample(triangle) for _ in range(points_count)]


def generateForPolygon(poly, intensity=100):
    triangles = [t for t in triangulate(poly) if poly.contains(t)]
    res_points = []
    for t in triangles:
        res_points.extend(generateForTriangle(t, intensity))
    return res_points
