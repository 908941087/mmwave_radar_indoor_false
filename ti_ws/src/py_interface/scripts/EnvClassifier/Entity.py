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


class Wall(Entity):

    def __init__(self, id, points):
        self.entity_id = id
        self.enhanced = False
        self.points = points
        self.width = None
        self.length = None

    def getInfo(self):
        return {"id": self.entity_id, "enhanced": self.enhanced, "width": self.width, "length": self.length}

    def show(self, plt):
        plt.plot(self.points)

    def enhance(self, cluster):
        pass

    def getWidth(self):
        return self.width

    def getLength(self):
        return self.length


class Furniture(Entity):

    def __init__(self, id):
        self.entity_id = id
        self.type = None
        self.is_enhanced = False

    def enhance(self, cluster):
        return self, cluster


class Door(Entity):

    class DoorState(Enum):
        OPEN = 0
        CLOSED = 1

    def __init__(self, id):
        self.entity_id = id
        self.state = DoorState.CLOSED
        self.is_enhanced = False

    def getState(self):
        return self.state

    def getLocation(self):
        pass

    def getCenter(self):
        pass

    def enhance(self, cluster):
        pass


class Noise(Entity):

    def __init__(self, id):
        self.entity_id = id
        self.center = None
        self.is_enhanced = False

    def getCenter(self):
        pass

    def getNaiveCenter(self):
        pass

    def enhance(self, cluster):
        pass

    def getInfo(self):
        pass