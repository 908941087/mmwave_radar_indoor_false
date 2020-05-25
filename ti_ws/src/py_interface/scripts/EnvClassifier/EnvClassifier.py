from Environment import Environment
from shapely.geometry import Polygon
from Cluster import Cluster, ClusterType
from shapely.ops import transform
from Entity import Wall, Furniture, Door, Noise
from PointCloudOperator import ClusterFit
from centerline.exceptions import TooFewRidgesError
from rtree import index
from sklearn.neighbors import KDTree
import numpy as np
from PointCloudOperator.ClusterFit import boneFit


class EnvClassifier(object):
    AREA_THRESHOLD = 0.25  # in square meter, area smaller than this threshold will be considered as noise
    MAX_WALL_WIDTH = 0.3
    MIN_WALL_LENGTH = 1.1
    RATIO_THRESHOLD = 5  # longer edge over shorter edge
    NOISE_POINTS_COUNT_THRESHOLD = 40
    MIN_DOOR_LENGTH = 0.5  # meters
    MAX_DOOR_LENGTH = 5  # meters

    def __init__(self):
        self.distances = {}  # cluster to cluster distance
        self._show_noise = True

    def classify(self, laser_clusters, mmwave_clusters):
        env = Environment()

        # for lc in laser_clusters:
        #     wall = boneFit(lc)
        #     env.register(wall, lc)
        single_lcs, lc_mc_matches, single_mcs = self.match(laser_clusters, mmwave_clusters)
        self.classifySingleLCs(single_lcs, env)
        self.classifyLCMCMatches(lc_mc_matches, env)
        self.classifySingleMCs(single_mcs, env)
        return env

    def match(self, laser_clusters, mmwave_clusters):
        """
        Match laser clusters(2D) with mmwave clusters(3D), some may have a match, some may not.
        :param laser_clusters: list of Cluster with 2D points inside
        :param mmwave_clusters: list of Cluster with 3D points inside
        :return: laser clusters with no mmwave cluster match, mmwave clusters with no laser cluster match, matched laser
        cluster and mmwave cluster pairs.
        """
        # convert 3d mmwave cluster to 2d clusters
        mc_2d = {}
        for c in mmwave_clusters:
            mc_2d[c.getId()] = Cluster(c.getId(), ClusterType.MMWAVE, 2, transform(lambda x, y, z=None: (x, y), c.getPoints()))

        # create rtree index using mmwave clusters for matching
        idx = index.Index()
        for mc in mc_2d.values():
            idx.insert(mc.getId(), mc.getBounds())

        # loop through laser clusters, seek for matching mmwave clusters
        # one laser cluster might match with multiple mmwave clusters
        single_lcs = []
        lc_mc_matches = []
        matched_mc_ids = []
        for lc in laser_clusters:
            neighbor_mc_ids = [i for i in idx.nearest(lc.getBounds(), 3)]
            matched_mcs = []
            lc_poly = lc.getConcaveHull()
            for mc_id in neighbor_mc_ids:
                mc = mc_2d[mc_id]
                if lc_poly.contains(mc.getCenter()):
                    matched_mcs.append(mc)
                    matched_mc_ids.append(mc_id)
                    continue
                intersection_area = mc.getConcaveHull().intersection(lc.getConcaveHull()).area
                if intersection_area / mc.getArea() > 0.8:
                    matched_mcs.append(mc)
                    matched_mc_ids.append(mc_id)
                    continue
            if len(matched_mcs) != 0:
                lc_mc_matches.append((lc, matched_mcs))
            else:
                single_lcs.append(lc)
        
        single_mcs = [mc for mc in mc_2d.values() if mc.getId() not in matched_mc_ids]
        return single_lcs, lc_mc_matches, single_mcs

    def classifyOneLC(self, lc):
        # env.register(Noise(lc.getId(), Polygon([[0, 0], [0, 1], [1, 0]])), lc)
        # use area and points count to recognize noise
        area = lc.getArea()
        if area < self.AREA_THRESHOLD and lc.getPointsCount() < self.NOISE_POINTS_COUNT_THRESHOLD:
            return Noise(lc.getId(), lc.getConcaveHull())

        # try to treat this cluster as wall, see if it fits well
        try:
            wall = ClusterFit.boneFit(lc)
            if wall.getWidth() > self.MAX_WALL_WIDTH or wall.getLength() < self.MIN_WALL_LENGTH or \
                    wall.getLength() / wall.getWidth() < self.RATIO_THRESHOLD:
                return Furniture(lc.getId(), lc.getConcaveHull())
            else:
                return wall
        except:
            return Furniture(lc.getId(), lc.getConcaveHull())

    def classifySingleLCs(self, lcs, env):
        """
        Classification for laser clusters with no matching mmwave scans.
        :param lcs: list of laser clusters
        :param env: type: Environment
        :return: None
        """
        if lcs is None:
            return

        for lc in lcs:
            entity = self.classifyOneLC(lc)
            env.register(entity, lc)

    def classifyLCMCMatches(self, lc_mc_matches, env):
        """
        Classification for laser cluster with matching mmwave scans.
        :param lc_mc_matches: list of (lc, [mc0, mc1, ..., mcn])
        :return: None
        """
        if lc_mc_matches is None:
            return None
        for match in lc_mc_matches:
            lc = match[0]
            mcs = match[1]
            lc_entity = self.classifyOneLC(lc)
            if isinstance(lc_entity, Noise):
                lc_entity = Furniture(lc.getId(), lc.getConcaveHull())
            env.register(lc_entity, lc)

    def classifySingleMCs(self, mcs, env):
        """
        Classification for mmwave cluster with no matching laser cluster, mostly doors
        :param mcs: list of mmwave clusters
        :param env: type: Environment
        :return: None
        """
        if mcs is None or len(mcs) == 0:
            return None
        possible_doors = self.findPossibleDoors(env)

        # create KDTree using door centers
        door_centers = [[d.getRepresentativePoint().x, d.getRepresentativePoint().y] for d in possible_doors]
        door_centers = np.array(door_centers).reshape(-1, 2)
        tree = KDTree(door_centers, leaf_size=2)

        # query the KDTree to find the nearest door of a mmwave cluster
        for mc in mcs:
            p = mc.getConcaveHull().representative_point()
            dist, ind = tree.query([[p.x, p.y]], k=1)
            nearest_door = possible_doors[ind[0][0]]
            if mc.getConcaveHull().contains(nearest_door.getRepresentativePoint()):
                env.register(nearest_door, mc)

    def findPossibleDoors(self, env):
        """
        Find possible doors in the environment. Possible doors means that there doesn't necessarily have
        to be a door exist.
        :param env: type: Environment
        :return: list of type Door, all possible doors
        """
        walls = [w for w in env.getEntities() if isinstance(w, Wall)]
        doors = []
        if len(walls) == 0:
            return doors

        # create RTree using walls
        idx = index.Index()
        for w in walls:
            idx.insert(w.getId(), w.getPolygon().bounds)

        # query for the nearest 3 walls for every wall and check if there exists a door
        max_entity_id = env.entity_count
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
                    doors.append(door)
                    max_entity_id += 1
        return doors
