from Entity import Wall, Furniture, TranspanrentObstacle, UnfinishedEntity
from MarkerGenerator import MarkerGenerator
from geometry_msgs import msg
from shapely.geometry import MultiPolygon, Polygon, LinearRing
import rospy

class Environment(object):

    def __init__(self):
        self.is_enhanced = False
        self.entity_cluster_map = {}
        self._show_unfinished = True
        self.entity_count = 0
        self.marker_generator = MarkerGenerator()

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
            elif isinstance(entity, TranspanrentObstacle):
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

    def generateInfoMarkers(self):
        pub_markers = []
        for entity in self.getEntities():
            if not self._show_unfinished and isinstance(entity, UnfinishedEntity):
                continue
            marker = self.marker_generator.generate_entity_info_marker(entity.getInfo())
            if marker is not None:
                pub_markers.append(marker)
        return pub_markers

    def generateShapeMarkers(self):
        pub_markers = []
        for entity in self.getEntities():
            if not self._show_unfinished and isinstance(entity, UnfinishedEntity):
                continue
            if hasattr(entity, "getPolygon"):
                poly = entity.getPolygon().simplify(tolerance=0.1, preserve_topology=False)
                try:
                    if isinstance(poly, MultiPolygon):
                        for p in list(poly):
                            count = 0
                            if len(p.exterior.coords) > 0:
                                marker_id = entity.getId() * 10000 + count
                                marker = self.marker_generator.generate_obstacle_bbox(marker_id, p.exterior.coords, entity.getHeight())
                                pub_markers.append(marker)
                                count += 1
                    elif isinstance(poly, Polygon):
                        marker_id = entity.getId()
                        marker = self.marker_generator.generate_obstacle_bbox(marker_id, poly.exterior.coords, entity.getHeight())
                        pub_markers.append(marker)
                    elif isinstance(poly, LinearRing):
                        marker_id = entity.getId()
                        marker = self.marker_generator.generate_obstacle_bbox(marker_id, poly.coords, entity.getHeight())
                        pub_markers.append(marker)
                    else:
                        print(type(poly))
                except (AttributeError, IndexError, ValueError):
                    rospy.loginfo("Encountered empty {0}. Skipping...".format(type(poly)))
        return pub_markers

    def generateTransparentObstacleMarkers(self):
        poly = msg.Polygon()
        try:
            transparent_obstacles = [e for e in self.getEntities() if isinstance(e, TranspanrentObstacle)]
            for to in transparent_obstacles:
                poly.points.extend([msg.Point32(p[0], p[1], 0.0) for p in to.getSegment().coords])
        except (AttributeError, IndexError, ValueError):
            rospy.loginfo("Encountered problem getting transparent obstacles. Skipping...")
        return poly
