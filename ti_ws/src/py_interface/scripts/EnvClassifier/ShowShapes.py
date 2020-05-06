def showLineString(line, plt):
    points = line.coords
    plt.plot([p[0] for p in points], [p[1] for p in points], 'ro-')