from enum import Enum

class Entity(object):

    def __init__(self, id):
        self.entity_id = id
        self.enhanced = False

    def show(self, plt):
        pass

    def getCenter(self):
        pass

    def getNaiveCenter(self):
        pass

    def enhance(self, cluster):
        pass

    def getInfo(self):
        pass

    def isEnhanced(self):
        return self.enhanced

    def getId(self):
        return self.entity_id


class Wall(Entity):

    def __init__(self, id, segments, width):
        self.entity_id = id
        self.enhanced = False
        self.segments = segments
        self.width = width
        self.length = None

    def getInfo(self):
        return {"id": self.entity_id, "enhanced": self.enhanced, "segments": self.segments, "width": self.width, "length": self.length}

    def show(self, plt):
        for segment in self.segments:
            points = [segment.p1, segment.p2]
            plt.plot([p[0] for p in points], [p[1] for p in points], 'ro-')

    def enhance(self, cluster):
        pass

    def getWidth(self):
        return self.width

    def getLength(self):
        if self.length is None:
            self.length = 0
            for segment in self.segments:
                self.length += segment.length
        return self.length

    def getCenter(self):
        pass

    def getNaiveCenter(self):
        pass


class Furniture(Entity):

    def __init__(self, id, polygon):
        self.entity_id = id
        self.polygon = None
        self.type = None
        self.is_enhanced = False

    def enhance(self, cluster):
        return self, cluster

    def getCenter(self):
        pass

    def getNaiveCenter(self):
        return self.polygon.centroid

    def show(self, plt):
        pass


class Door(Entity):

    class DoorState(Enum):
        OPEN = 0
        CLOSED = 1

    def __init__(self, id, ends):
        self.entity_id = id
        self.state = DoorState.CLOSED
        self.is_enhanced = False
        self.ends = ends

    def getState(self):
        return self.state

    def getLocation(self):
        return self.getCenter()

    def getCenter(self):
        return ends[0].midpoint(ends[1])

    def enhance(self, cluster):
        pass


class Noise(Entity):

    def __init__(self, id, polygon):
        self.entity_id = id
        self.center = None
        self.polygon = polygon
        self.is_enhanced = False

    def getCenter(self):
        pass

    def getNaiveCenter(self):
        return self.polygon.centroid

    def enhance(self, cluster):
        pass

    def getInfo(self):
        pass

    def show(self, plt):
        pass