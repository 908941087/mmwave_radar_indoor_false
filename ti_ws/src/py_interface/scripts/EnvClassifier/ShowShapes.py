def showLineString(line, plt):
    points = line.coords
    plt.plot([p[0] for p in points], [p[1] for p in points], 'ro-')


def showPolygon(polygon, plt):
    x, y = polygon.exterior.xy
    plt.plot(x, y, 'bo-')