from Environment import Environment
from shapely.geometry import Polygon, Point, MultiPolygon, LineString, LinearRing, MultiLineString
from Cluster import Cluster, ClusterType
from shapely.ops import transform
from Entity import Wall, Furniture, TranspanrentObstacle, UnfinishedEntity
from PointCloudOperator import ClusterFit
from rtree import index
from sklearn.neighbors import KDTree
import numpy as np
from timer import timer
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
        self.mmwave_clusters = []

    def classify(self, laser_clusters, mmwave_clusters, robot_pose):
        self.mmwave_clusters = mmwave_clusters
        env = Environment()

        # for lc in laser_clusters:
        #     wall = boneFit(lc)
        #     env.register(wall, lc)
        single_lcs, lc_mc_matches, single_mcs = self.match(laser_clusters, mmwave_clusters)
        self.classifySingleLCs(single_lcs, env)
        self.classifyLCMCMatches(lc_mc_matches, env)
        self.classifyMCs(single_mcs, lc_mc_matches, env, robot_pose)
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
                if nearest_lc_id == lc.getId():
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

    def classifyMCs(self, single_mcs, lc_mc_matches, env, robot_pose):
        """
        Classification for mmwave cluster, mostly doors
        :param single_mcs: list of mmwave clusters with no lc match
        :param lc_mc_matches: list of mmwave clusters with one lc match
        :param env: type: Environment
        :param robot_pose: robot odometry data
        :return: None
        """
        if single_mcs is None or len(single_mcs) == 0:
            return None

        # use robot pose to find noise from single single_mcs
        pose = Point(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y)
        wall_polygons = []
        for entity in env.getEntities():
            if isinstance(entity, Wall):
                poly = entity.getPolygon()
                if isinstance(poly, MultiPolygon):
                    wall_polygons.extend([p for p in poly])
                elif isinstance(poly, LinearRing):
                    wall_polygons.append(Polygon(poly))
                elif isinstance(poly, Polygon):
                    wall_polygons.append(poly)
        wall_obstacle = MultiPolygon(wall_polygons)
        noise_mc_ids = [mc.getId() for mc in single_mcs if LineString([mc.getPoints()[0], pose]).intersects(wall_obstacle)]
        mcs = [mc for mc in single_mcs if mc.getId() not in noise_mc_ids]

        # filter out matched mcs that have a high overlap with its respective lc
        for pair in lc_mc_matches:
            lc = pair[0]
            lmcs = pair[1]  # local mcs
            for mc in lmcs:
                if not lc.getConcaveHull().contains(mc.getCenter()):
                    mcs.append(mc)

        possible_doors = self.findPossibleDoors(env)

        # create KDTree using door centers
        if len(possible_doors) == 0:
            return None
        door_centers = [[d.getRepresentativePoint().x, d.getRepresentativePoint().y] for d in possible_doors]
        door_centers = np.array(door_centers).reshape(-1, 2)
        tree = KDTree(door_centers, leaf_size=2)

        # query the KDTree to find the nearest door of a mmwave cluster
        for mc in mcs:
            p = mc.getConcaveHull().centroid
            dist, ind = tree.query([[p.x, p.y]], k=1)
            nearest_door = possible_doors[ind[0][0]]
            if mc.getConcaveHull().buffer(0.4).contains(nearest_door.getRepresentativePoint()):
                # mc seems to match with this door, adjust door position according to mc position
                door_line = nearest_door.getSegment()
                mc_line = ClusterFit.lineFit(mc.getPoints())
                mc_line_segment = mc_line.difference(wall_obstacle)
                segment = self.findSegment(mc_line_segment, door_line)
                if 0.5 < segment.length / nearest_door.getSegment().length < 1.5:
                    nearest_door.segment = segment
                    nearest_door.setHeight(self.getHeightFromMc(mc))
                    env.register(nearest_door, mc)
            elif mc in single_mcs:
                entity = self.classify2DCluster(mc)
                entity.setHeight(self.getHeightFromMc(mc))
                env.register(entity, mc)

    @staticmethod
    def findSegment(source_line, target_line):
        """
        Find a line segment comprise of two points for source line that resembles target line most.
        :param source_line: could be MultiLineString, LineString
        :param target_line: LineString with two vertices
        :return: LineString with two vertices
        """
        def findSegmentFromLineString(source, target):
            if len(source.coords) == 2:
                return source, source.distance(target)
            segments = [LineString(source.coords[i:i + 2]) for i in range(len(source.coords) - 1)]
            min_distance = float('inf')
            nearest_s = None
            for s in segments:
                current_distance = s.distance(target)
                if current_distance < min_distance:
                    min_distance = current_distance
                    nearest_s = s
            return nearest_s, min_distance

        if isinstance(source_line, MultiLineString):
            nearest_line = None
            min_dist = float('inf')
            for line in source_line:
                current_nearest, current_min_dist = findSegmentFromLineString(line, target_line)
                if current_min_dist < min_dist:
                    min_dist = current_min_dist
                    nearest_line = current_nearest
            result_line = nearest_line
        else:  # source line is of type LineString
            result_line, min_dist = findSegmentFromLineString(source_line, target_line)
        return result_line

    def findPossibleDoors(self, env):
        """
        Find possible doors in the environment. **Possible** doors means that there doesn't necessarily have
        to be a door.
        :param env: type: Environment
        :return: list of type TranspanrentObstacle, all possible doors
        """
        doors = []
        if len(env.getEntities()) == 0:
            return doors

        # create RTree using entities
        idx = index.Index()
        for e in env.getEntities():  # Wall, Furniture, UnfinishedEntity
            idx.insert(e.getId(), e.getPolygon().bounds)

        # query for the nearest 3 entities for every entity and check if there exists a door
        max_entity_id = env.entity_count
        queried_pairs = []
        for e in env.getEntities():
            neighbor_entity_ids = [i for i in idx.nearest(e.getPolygon().bounds, 4)][1:]
            for n in neighbor_entity_ids:
                id_pair = [n, e.getId()]
                id_pair.sort()
                if id_pair in queried_pairs:
                    continue
                queried_pairs.append(id_pair)
                door = ClusterFit.doorFit(e, env.getEntity(n))

                # check if door is valid
                length = door.getLength()
                if self.MIN_DOOR_LENGTH <= length <= self.MAX_DOOR_LENGTH:
                    door.entity_id = max_entity_id
                    doors.append(door)
                    max_entity_id += 1
        return doors

    def getHeightFromMcs(self, mcs):
        heights = [self.getHeightFromMc(mc) for mc in mcs]
        return np.average(heights)

    def getHeightFromMc(self, mc):
        height = 0.0
        for p in self.mmwave_clusters[mc.getId()].getPoints():
            if p.z > height:
                height = p.z
        return height
