from enum import Enum

class Entity(object):

    def __init__(self, id):
        self.entity_id = id
        self.generated = False

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


class Wall(Entity):

    def __init__(self, id):
        self.entity_id = id
        self.generated = False
        self.width = None
        self.length = None

    def getPoints(self):
        pass

    def getInfo(self):
        pass

    def show(self, plt):
        pass


class Furniture(Entity):

    def __init__(self, id):
        self.entity_id = id
        self.type = None


class Door(Entity):

    class DoorState(Enum):
        OPEN = 0
        CLOSED = 1

    def __init__(self, id):
        self.entity_id = id
        self.state = DoorState.CLOSED