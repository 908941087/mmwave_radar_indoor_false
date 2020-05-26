from Entity import Wall, Furniture, Door, UnfinishedEntity


class Environment(object):

    def __init__(self):
        self.is_enhanced = False
        self.entity_cluster_map = {}
        self._show_unfinished = False
        self.entity_count = 0

    def register(self, entity, cluster):
        entity.id = self.entity_count
        self.entity_count += 1
        self.entity_cluster_map[entity.getId()] = [entity, cluster]

    def getEntities(self):
        return [i[0] for i in self.entity_cluster_map.values()]

    def getClusters(self):
        return [i[1] for i in self.entity_cluster_map.values() if i[1] is not None]

    def getEntity(self, eid):
        return self.entity_cluster_map[eid][0]

    def getCluster(self, eid):
        return self.entity_cluster_map[eid][1]

    def enhance(self):
        """
        Returns an enhanced version of this Environment.
        """
        if self.is_enhanced:
            raise Exception("Environment is already enhanced.")
        env = Environment()
        for pair in self.entity_cluster_map.values():
            new_entity, new_cluster = pair[0].enhance(pair[1])
            try:
                env.register(new_entity, new_cluster)
            except AttributeError:
                pass
        env.is_enhanced = True
        return env

    def getPoints(self):
        res_points = []
        for cluster in self.getClusters():
            if cluster is not None: res_points.extend(cluster.getPoints())
        return res_points

    def getEntityInfos(self):
        return [entity.getInfo() for entity in self.getEntities()]

    def show(self, plt):
        for pair in self.entity_cluster_map.values():
            entity = pair[0]
            cluster = pair[1]
            c = 'r'
            if isinstance(entity, Wall):
                c = 'grey'
            elif isinstance(entity, Furniture):
                c = 'blue'
            elif isinstance(entity, UnfinishedEntity):
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

    def showEntityShapes(self, plt):
        for entity in self.getEntities():
            entity.showShape(plt)

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
            plt.text(center.x, center.y, tag, style='italic', fontsize=14,
                     bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 5})

    def __repr__(self):
        return str(self)

    def __str__(self):
        return str(len(self.entity_cluster_map)) + " entities and clusters."

    def generateInfoMarkers(self, duration=5.0):
        mark_index = 0
        pub_markers = []
        for entity in self.getEntities():
            if not self._show_unfinished and isinstance(entity, UnfinishedEntity):
                continue
            marker = entity.getInfoMarker(mark_index, duration)
            if marker is not None:
                pub_markers.append(marker)
                mark_index += 1
        return pub_markers

    def generateShapeMarkers(self, duration=5.0):
        mark_index = 0
        pub_markers = []
        for entity in self.getEntities():
            if not self._show_unfinished and isinstance(entity, UnfinishedEntity):
                continue
            marker = entity.getShapeMarker(mark_index, duration)
            if marker is not None:
                pub_markers.append(marker)
                mark_index += 1
        return pub_markers
