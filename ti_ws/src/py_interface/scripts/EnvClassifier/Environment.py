from Entity import Wall

class Environment(object):

    def __init__(self):
        self.is_enhanced = False
        self.entity_cluster_map = {}

    def register(self, entity, cluster):
        self.entity_cluster_map[cluster.getId(): [entity, cluster]]

    def getEntities(self):
        return [i[0] for i in self.entity_cluster_map.values()]

    def getClusters(self):
        return [i[1] for i in self.entity_cluster_map.values()]
        
    def enhance(self):
        if self.is_enhanced: raise Exception("Environment is already enhanced.")
        env = Environment()
        env.is_enhanced = True
        for entry in self.entity_cluster_map.values():
            new_entity, new_cluster = entry[0].enhance(entry[1])
            env.register(new_entity, new_cluster)
        return env

    def getPoints(self):
        res_points = []
        for cluster in self.getClusters():
            if cluster is not None: res_points.extend(cluster.getPoints())
        return res_points

    def show(self, plt):
        for entity in self.getEntities():
            entity.show(plt)

    def showWalls(self, plt):
        for entity in self.getEntities():
            if isinstance(entity, Wall):
                entity.show(plt)