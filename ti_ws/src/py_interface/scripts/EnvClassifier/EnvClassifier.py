from Environment import Environment
from shapely.geometry import Polygon, Point, MultiPolygon, LineString, LinearRing, MultiLineString
from Cluster import Cluster, ClusterType
from shapely.ops import transform, nearest_points
from Entity import Wall, Furniture, TranspanrentObstacle, UnfinishedEntity
from PointCloudOperator import ClusterFit
from rtree import index
from sklearn.neighbors import KDTree
import numpy as np
from timer import timer
from PointCloudOperator.ClusterFit import boneFit
import rospy


class EnvClassifier(object):
    AREA_THRESHOLD = 0.25  # in square meter, area smaller than this threshold will be considered as noise
    MAX_WALL_WIDTH = 0.3
    MIN_WALL_LENGTH = 1.1
    RATIO_THRESHOLD = 5  # longer edge over shorter edge
    NOISE_POINTS_COUNT_THRESHOLD = 40
    MIN_DOOR_LENGTH = 0.5  # meters
    MIN_END_TO_WALL_LENGTH = 0.8  # meters
    MIN_DOOR_AREA = 0.4  # square meters

    def __init__(self):
        self.distances = {}  # cluster to cluster distance
        self._show_noise = True
        self.mmwave_clusters_dict = None

    def classify(self, laser_clusters, mmwave_clusters):
        self.mmwave_clusters_dict = {mc.getId(): mc for mc in mmwave_clusters}
        env = Environment()

        # for lc in laser_clusters:
        #     wall = boneFit(lc)
        #     env.register(wall, lc)
        single_lcs, lc_mc_matches, single_mcs = self.match(laser_clusters, mmwave_clusters)
        self.classifySingleLCs(single_lcs, env)
        self.classifyLCMCMatches(lc_mc_matches, env)
        self.classifySingleMCs(single_mcs, env)
        return env

    @timer
    def match(self, laser_clusters, mmwave_clusters):
        """
        Match laser clusters(2D) with mmwave clusters(3D), some may have a match, some may not.
        :param laser_clusters: list of Cluster with 2D points inside
        :param mmwave_clusters: list of Cluster with 3D points inside
        :return: laser clusters with no mmwave cluster match, 2d mmwave clusters, matched laser
        cluster and mmwave cluster pairs.
        """
        # convert 3d mmwave cluster to 2d clusters
        mc_2d = {}
        for c in mmwave_clusters:
            mc_2d[c.getId()] = Cluster(c.getId(), ClusterType.MMWAVE, 2, transform(lambda x, y, z=None: (x, y), c.getPoints()))

        lc_2d = {lc.getId(): lc for lc in laser_clusters}

        # create rtree index for mmwave clusters and laser clusters
        mc_idx = index.Index()
        for mc in mc_2d.values():
            mc_idx.insert(mc.getId(), mc.getBounds())

        lc_idx = index.Index()
        for lc in lc_2d.values():
            lc_idx.insert(lc.getId(), lc.getBounds())

        # loop through laser clusters, seek for matching mmwave clusters
        # one laser cluster might match with multiple mmwave clusters
        single_lcs = []
        lc_mc_matches = []
        matched_mc_ids = []
        for lc in laser_clusters:
            neighbor_mc_ids = [i for i in mc_idx.intersection(lc.getBounds())]
            matched_mcs = []
            for mc_id in neighbor_mc_ids:
                mc = mc_2d[mc_id]
                nearest_lc_id = [i for i in lc_idx.nearest(mc.getBounds(), 1)][0]
                if nearest_lc_id == lc.getId() and \
                        lc.getConcaveHull().buffer(0.1).contains(mc.getCenter()):
                    matched_mcs.append(mc)
                    matched_mc_ids.append(mc_id)
            if len(matched_mcs) != 0:
                lc_mc_matches.append((lc, matched_mcs))
            else:
                single_lcs.append(lc)
        
        single_mcs = [mc for mc in mc_2d.values() if mc.getId() not in matched_mc_ids]
        return single_lcs, lc_mc_matches, single_mcs

    def classify2DCluster(self, lc):
        # env.register(UnfinishedEntity(lc.getId(), Polygon([[0, 0], [0, 1], [1, 0]])), lc)
        # use area and points count to recognize noise
        area = lc.getArea()
        if area < self.AREA_THRESHOLD and lc.getPointsCount() < self.NOISE_POINTS_COUNT_THRESHOLD:
            return UnfinishedEntity(lc.getConcaveHull())

        # try to treat this cluster as wall, see if it fits well
        try:
            wall = ClusterFit.boneFit(lc)
            if wall.getWidth() > self.MAX_WALL_WIDTH or wall.getLength() < self.MIN_WALL_LENGTH or \
                    wall.getLength() / wall.getWidth() < self.RATIO_THRESHOLD:
                return Furniture(lc.getConcaveHull())
            else:
                return wall
        except:
            return Furniture(lc.getConcaveHull())

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
            entity = self.classify2DCluster(lc)
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
            lc_entity = self.classify2DCluster(lc)
            if isinstance(lc_entity, UnfinishedEntity):
                lc_entity = Furniture(lc.getConcaveHull())
            lc_entity.setHeight(self.getHeightFromMcs(mcs))
            env.register(lc_entity, lc)

    def classifySingleMCs(self, single_mcs, env):
        """
        Classification for mmwave cluster, mostly doors
        :param single_mcs: list of mmwave clusters with no lc match
        :param env: type: Environment
        :return: None
        """
        if single_mcs is None or len(single_mcs) == 0:
            return None

        # build RTree using entities in env
        entities = [entity for entity in env.getEntities()]
        entity_idx = index.Index()
        for e in entities:
            entity_idx.insert(e.getId(), e.getPolygon().bounds)

        # filter out mcs that are too small
        candidate_mcs = [mc for mc in single_mcs if mc.getArea() > self.MIN_DOOR_AREA]

        # try to fit mc as wall, if valid, extend the ends to the nearest wall
        for mc in candidate_mcs:
            try:
                temp_w = ClusterFit.boneFit(mc)
            except ValueError:
                fur = Furniture(mc.getConcaveHull())
                fur.setHeight(self.getHeightFromMc(self.mmwave_clusters_dict[mc.getId()]))
                env.register(fur, mc)
                continue

            if temp_w.getLength() >= self.MIN_DOOR_LENGTH:
                bounds = mc.getBounds()
                length = max([bounds[2] - bounds[0], bounds[3] - bounds[1]])
                if 0.7 <= length / temp_w.getLength() <= 1.3:
                    line = ClusterFit.lineFit(mc.getPoints())
                    ends = [Point(t) for t in list(line.coords)]
                    nearest_entities = []
                    for end in ends:
                        neighbor_entity_ids = [i for i in entity_idx.nearest(end.bounds, 3)]
                        distances = {end.distance(env.getEntity(eid).getPolygon()): eid for eid in neighbor_entity_ids}
                        min_dist = min(distances.keys())
                        nearest_entities.append(env.getEntity(distances[min_dist]))
                    if nearest_entities[0] != nearest_entities[1]:
                        end_entity_dist = [line.distance(e.getPolygon()) for e in nearest_entities]
                        ends_validity = [0 <= d <= self.MIN_END_TO_WALL_LENGTH for d in end_entity_dist]
                        if False not in ends_validity:
                            mid_point = line.centroid
                            new_ends = [nearest_points(mid_point, e.getPolygon())[1] for e in nearest_entities]
                            rospy.logerr(str([list(p.coords) for p in new_ends]))
                            env.register(TranspanrentObstacle(LineString(new_ends)), mc)
                            continue
            fur = Furniture(mc.getConcaveHull())
            fur.setHeight(self.getHeightFromMc(self.mmwave_clusters_dict[mc.getId()]))
            env.register(fur, mc)

    def getHeightFromMcs(self, mcs):
        heights = [self.getHeightFromMc(mc) for mc in mcs]
        return np.average(heights)

    def getHeightFromMc(self, mc):
        height = 0.0
        for p in self.mmwave_clusters_dict[mc.getId()].getPoints():
            if p.z > height:
                height = p.z
        return height
