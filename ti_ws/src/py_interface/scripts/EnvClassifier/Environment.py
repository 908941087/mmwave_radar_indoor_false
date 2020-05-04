from Entity import Wall, Furniture, Door, Noise

class Environment(object):

    def __init__(self):
        self.is_enhanced = False
        self.entity_cluster_map = {}

    def register(self, entity, cluster):
        if entity.getId() != cluster.getId():
            raise Exception("Entity and cluster must have the same id.")
        self.entity_cluster_map[cluster.getId()] = [entity, cluster]

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
        for pair in self.entity_cluster_map.values():
            entity = pair[0]
            cluster = pair[1]
            c = 'r'
            if isinstance(entity, Wall):
                c = 'grey'
            elif isinstance(entity, Furniture):
                c = 'blue'
            elif isinstance(entity, Noise):
                c = 'r'
            elif isinstance(entity, Door):
                entity.show(plt)
                continue
            cluster.show(plt, c)
            entity.show(plt)

    def showEntities(self, plt):
        for entity in self.getEntities():
            entity.show(plt)

    def showClusters(self, plt):
        for cluster in self.getClusters():
            cluster.show(plt)

    def showWalls(self, plt):
        for entity in self.getEntities():
            if isinstance(entity, Wall):
                entity.show(plt)

    def showEntityTags(self, plt):
        for entity in self.getEntities():
            info = entity.getInfo()
            center = entity.getRepresentativePoint()
            tag = ""
            for key in info.keys():
                tag += str(key) + ": " + str(info[key]) + "\n"
            tag = tag.rstrip('\n')
            plt.text(center.x, center.y, tag, style='italic', fontsize=6, bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 5})

    def __repr__(self):
        return str(self)

    def __str__(self):
        return str(len(self.entity_cluster_map)) + " entities and clusters."