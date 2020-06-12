from shapely.geometry import LineString, Point

def getExtendedLine(line, distance):
    """
    Extrapolate a line. If distance > 0, extrapolate to the right hand side, else extrapolate to the left hand side.
    :param line: a LineString with two point
    :param distance: float number
    :return: a new line with the same direction and start point
    """
    if len(line.coords) != 2:
        raise ValueError("Line must consist of two points.")
    ends = [Point(p) for p in list(line.coords)]
    sorted(ends, cmp=pointCompare)
    if distance > 0:
        start = ends[0]
        end = ends[1]
    else:
        start = ends[1]
        end = ends[0]
    distance = abs(distance)
    length = line.length
    ratio = distance / length
    new_end = Point(ratio * (end.x - start.x) + end.x, ratio * (end.y - start.y) + end.y)
    return LineString([start, new_end])


def pointCompare(p1, p2):
    if p1.x != p2.x:
        return p1.x > p2.x
    else:
        return p1.y > p2.y


if __name__ == "__main__":
    l = LineString([[0, 0], [1, 1]])
    new_l = getExtendedLine(l, 0)
    print(list(new_l.coords))
