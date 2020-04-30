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

    def enhance(self):
        pass

    def getInfo(self):
        pass

    def isEnhanced(self):
        return self.enhanced


class Wall(Entity):

    def __init__(self, id):
        self.entity_id = id
        self.enhanced = False
        self.width = None
        self.length = None

    def getPoints(self):
        pass

    def getInfo(self):
        pass

    def show(self, plt):
        pass

    def enhance(self):
        pass

    def getWidth(self):
        pass

    def getLength(self):
        pass


class Furniture(Entity):

    def __init__(self, id):
        self.entity_id = id
        self.type = None
        self.is_enhanced = False

    def enhance(self):
        pass


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

    def enhance(self):
        pass


class Noise(Entity):

    def __init__(self, id):
        self.entity_id = id
        self.center = None
        self.is_enhanced = False

    def getCenter(self):
        pass

    def enhance(self):
        pass

    def getNaiveCenter(self):
        pass

    def enhance(self):
        pass

    def getInfo(self):
        pass