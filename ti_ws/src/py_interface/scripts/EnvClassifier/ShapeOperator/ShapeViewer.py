from shapely.geometry import Polygon, MultiPolygon


def showLineString(line, plt, style='ro-'):
    points = line.coords
    plt.plot([p[0] for p in points], [p[1] for p in points], style)


def showSinglePolygon(polygon, plt, style='bo-'):
    x, y = polygon.exterior.xy
    plt.plot(x, y, style)


def showPolygon(polygon, plt, style='bo-'):
    if isinstance(polygon, Polygon):
        showSinglePolygon(polygon, plt, style)
    elif isinstance(polygon, MultiPolygon):
        for poly in list(polygon):
            showSinglePolygon(poly, plt, style)
