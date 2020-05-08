from Entity import Wall, Furniture, Door, Noise
import rospy
from visualization_msgs.msg import Marker


class Environment(object):

    def __init__(self):
        self.is_enhanced = False
        self.entity_cluster_map = {}
        self._show_noise = True
        self.pub_markers = []

    def register(self, entity, cluster):
        if entity.getId() != cluster.getId():
            raise Exception("Entity and cluster must have the same id.")
        self.entity_cluster_map[cluster.getId()] = [entity, cluster]

    def getEntities(self):
        return [i[0] for i in self.entity_cluster_map.values()]

    def getClusters(self):
        return [i[1] for i in self.entity_cluster_map.values()]

    def getEntity(self, id):
        return self.entity_cluster_map[id][0]

    def getCluster(self, id):
        return self.entity_cluster_map[id][1]

    def enhance(self):
        """
        Returns an enhanced version of this Environment.
        """
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
            plt.text(center.x, center.y, tag, style='italic', fontsize=6,
                     bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 5})

    def __repr__(self):
        return str(self)

    def __str__(self):
        return str(len(self.entity_cluster_map)) + " entities and clusters."

    def generate_markers(self, duration=5.0):
        mark_index = 0
        self.pub_markers = []
        for entity in self.getEntities():
            if not self._show_noise and isinstance(entity, Noise):
                continue
            t_marker = Marker()
            t_marker.header.frame_id = "/map"
            t_marker.header.stamp = rospy.Time.now()
            t_marker.ns = "cluster_class"

            t_marker.id = mark_index
            mark_index += 1
            # Type
            t_marker.type = Marker.TEXT_VIEW_FACING
            loc = entity.getRepresentativePoint()
            t_marker.text = entity.getInfo()["Name"] + "\n" + "x: " + str(round(loc.x, 3)) + "\n" + "y: " + str(
                round(loc.y, 3))
            # Size
            t_marker.scale.x = 0.3
            t_marker.scale.y = 0.3
            t_marker.scale.z = 0.3

            # ADD/DELETE
            t_marker.action = Marker.MODIFY

            # Pose
            t_marker.pose.position.x = loc.x
            t_marker.pose.position.y = loc.y
            t_marker.pose.position.z = 0.2
            t_marker.pose.orientation.x = 0.0
            t_marker.pose.orientation.y = 0.0
            t_marker.pose.orientation.z = 0.0
            t_marker.pose.orientation.w = 1.0

            # Color
            t_marker.color.r = 0.0
            t_marker.color.g = 1.0
            t_marker.color.b = 0.5
            t_marker.color.a = 1.0

            t_marker.lifetime = rospy.Duration(duration)
            self.pub_markers.append(t_marker)
        return self.pub_markers
