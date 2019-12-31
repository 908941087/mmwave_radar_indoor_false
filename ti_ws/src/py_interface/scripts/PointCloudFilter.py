from math import sqrt, pow
from Frame import *


def find_match_in_frame(source_point, frame):
    neighbor_threshold = 0.1  # FIXME: need revise
    match = []  # list to store matched points in a frame
    for p in frame:
        if dist(p, source_point) <= neighbor_threshold:
            match.append(p)
    return match


def dist(p1, p2):
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))


def find_neighbors(first, second):
    match_data_frame = []
    for point in first:
        match_data_frame.extend(find_match_in_frame(point, second))
    first.points.clear()
    first.points = match_data_frame
