def showLineString(line, plt, style='ro-'):
    points = line.coords
    plt.plot([p[0] for p in points], [p[1] for p in points], style)


def showPolygon(polygon, plt, style='bo-'):
    x, y = polygon.exterior.xy
    plt.plot(x, y, style)