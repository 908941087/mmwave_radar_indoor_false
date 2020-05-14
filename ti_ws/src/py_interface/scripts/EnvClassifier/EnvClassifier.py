from Environment import Environment
from Entity import Wall, Furniture, Door, Noise
from PointCloudOperator import ClusterFit
from centerline.exceptions import TooFewRidgesError
from rtree import index


class EnvClassifier(object):
    AREA_THRESHOLD = 0.25  # in square meter, area smaller than this threshold will be considered as noise
    DENSITY_THRESHOLD = 120  # lowest points per square meter
    MAX_WALL_WIDTH = 0.5
    MIN_WALL_LENGTH = 1.1
    RATIO_THRESHOLD = 3  # longer edge over shorter edge
    NOISE_POINTS_COUNT_THRESHOLD = 40
    DENSITY_PER_SQUARE_METER_THRESHOLD = 20000
    MIN_ISOLATION_DIST = 0.7  # meters
    MIN_DOOR_LENGTH = 0.3  # meters
    MAX_DOOR_LENGTH = 2  # meters

    def __init__(self):
        self.distances = {}  # cluster to cluster distance
        self._show_noise = False
        
    def classify(self, clusters):
        if clusters is None:
            return
        env = Environment()
        self.distances = {}

        # create entities that is matched with clusters, eg: Wall, Furniture, Noise
        for i in range(len(clusters)):
            cluster = clusters[i]

            # use area and density to recognize noise
            area = cluster.getArea()
            density = cluster.getDensity()
            if (area < self.AREA_THRESHOLD or density < self.DENSITY_THRESHOLD) and \
                    cluster.getPointsCount() < self.NOISE_POINTS_COUNT_THRESHOLD and density / area < self.DENSITY_PER_SQUARE_METER_THRESHOLD:
                env.register(Noise(cluster.getId(), cluster.getConcaveHull()), cluster)
                continue

            # try to treat this cluster as wall, see if it fits well
            try:
                wall = ClusterFit.boneFit(cluster)
                if wall.getWidth() > self.MAX_WALL_WIDTH or wall.getLength() < self.MIN_WALL_LENGTH or \
                        wall.getLength() / wall.getWidth() < self.RATIO_THRESHOLD:
                    env.register(Furniture(cluster.getId(), cluster.getConcaveHull()), cluster)
                    continue
                else:
                    env.register(wall, cluster)
            except:
                print("Cluster {0} encounters TooFewRidgesError.".format(cluster.getId()))
                env.register(Furniture(cluster.getId(), cluster.getConcaveHull()), cluster)

        # create entities that is determined by the relationship of other entities, eg: Door
        walls = [w for w in env.getEntities() if isinstance(w, Wall)]

        # create RTree using walls
        idx = index.Index()
        for w in walls:
            idx.insert(w.getId(), w.getPolygon().bounds)

        # query for the nearest 3 walls for every wall and check if there exists a door
        max_entity_id = len(clusters)
        queried_pairs = []
        for w in walls:
            neighbor_wall_ids = [i for i in idx.nearest(w.getPolygon().bounds, 4)][1:]
            for n in neighbor_wall_ids:
                id_pair = [n, w.getId()]
                id_pair.sort()
                if id_pair in queried_pairs:
                    continue
                queried_pairs.append(id_pair)
                door = ClusterFit.doorFit(w, env.getEntity(n))

                # check if door is valid
                length = door.getLength()
                if self.MIN_DOOR_LENGTH <= length <= self.MAX_DOOR_LENGTH:
                    door.entity_id = max_entity_id
                    env.register(door, None)
                    max_entity_id += 1

        return env
