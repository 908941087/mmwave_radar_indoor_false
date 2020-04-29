class Environment(object):

    def __init__(self):
        self.entities = []
        self.is_enhanced = False

    def add(self, entity):
        self.entities.append(entity)
        
    def enhance(self):
        pass

    def getPoints(self):
        pass

    def show(self, plt):
        pass

    def showWalls(self, plt):
        pass