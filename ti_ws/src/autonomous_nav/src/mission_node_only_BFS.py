#!/usr/bin/env python

# ROS imports
import roslib
import rospy
import numpy as np
import math

#ROS messages
from geometry_msgs.msg import Pose2D, Point
from autonomous_nav.msg import PotentialGrid
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from threading import Lock

from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from collections import deque

from autonomous_nav.srv import ForceFind, ForceFindRequest, ForceFindResponse

def angle_wrap(ang):
    """
    Return the angle normalized between [-pi, pi].
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang


class WaypointHistory:
    def __init__(self):
        self.wp_history = None
        self.hist_size = 70
        self.thres = 0.35
        
    def isWaypointInHistory(self , msg_pose):
        if self.wp_history is None:
            return False

        waypoints_hist = self.wp_history - np.array([msg_pose.x, msg_pose.y])
        eucl_dist = np.sqrt(np.sum(np.power(waypoints_hist, 2.0), axis=1)) #axis = 1 -> sum the rows

        # Computing Eucleadean Distance by checking Threshold
        return np.min(eucl_dist) < self.thres

        
    def addWaypointToHistory(self, msg_pose ):
        if self.wp_history is None:
            self.wp_history = np.array([[msg_pose.x, msg_pose.y]])
        else:
            self.wp_history = np.vstack((self.wp_history, 
                                         np.array([[msg_pose.x, msg_pose.y]])))
            
            if self.wp_history.shape[0] > self.hist_size:
                self.wp_history = np.delete( self.wp_history , (0), axis=0 )


class Pixel:
    x = 0
    y = 0
    def __init__(self, x , y):
        self.x = x
        self.y = y


class PotentialMapMaker:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_pixel_i = 0
        self.robot_pixel_j = 0
        self.param_odom = "/odom"
        self.param_polygon_map = "/map"
        self.param_inflation_radius = 8
        self.mutex = Lock()
        self.map = None

        rospy.Subscriber(self.param_odom, Odometry, self.odometryCallback)
        rospy.Subscriber(self.param_polygon_map, OccupancyGrid, self.mapCallback)
        self.pub = rospy.Publisher("potential_map", PotentialGrid, queue_size=1)


    def odometryCallback(self, msg):
        self.mutex.acquire()
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.mutex.release()

    
    def mapCallback(self, msg):
        self.mutex.acquire()

        self.map = msg
        self.map.data = list(msg.data)
        inflation_queue = deque()
        for i in range(0, msg.info.width):
            for j in range(0, msg.info.height):
                if msg.data[j * msg.info.width + i] == 100:
                    self.map.data[j * msg.info.width + i] = 2
                    inflation_queue.append(Pixel(i, j))
                elif msg.data[j * msg.info.width + i] == -1:
                    self.map.data[j * msg.info.width + i] = 1
                else:
                    self.map.data[j * msg.info.width + i] = 0

        inflation_queue2 = deque()
        
        for n in range(0, self.param_inflation_radius):
            inflation_queue2 = deque()

            while len(inflation_queue) > 0:
                temp = inflation_queue.popleft()
                for i in range(max(temp.x - 1, 0), min(temp.x + 2, msg.info.width)):
                    for j in range(max(temp.y - 1, 0), min(temp.y + 2, msg.info.height)):
                        if self.map.data[j * msg.info.width + i] <= 1:
                            self.map.data[j * msg.info.width + i] = 2
                            inflation_queue2.append(Pixel(i, j))
            inflation_queue = inflation_queue2
        self.robot_pixel_i = int((self.robot_x - msg.info.origin.position.x)/msg.info.resolution)
        self.robot_pixel_j = int((self.robot_y - msg.info.origin.position.y)/msg.info.resolution)

        if self.robot_pixel_i < 0 or self.robot_pixel_i > msg.info.width or self.robot_pixel_j < 0 or self.robot_pixel_j > msg.info.height:
            rospy.logwarn("Robot outside of projected_map")
            self.mutex.release()
            return
        
        self.map.data[self.robot_pixel_j * msg.info.width + self.robot_pixel_i] = 3
        queue = deque()
        queue.append(Pixel(self.robot_pixel_i, self.robot_pixel_j))

        while len(queue) > 0:
            temp = queue.popleft()
            for i in range(max(temp.x - 5, 0), min(temp.x + 5, msg.info.width)):
                    for j in range(max(temp.y - 5, 0), min(temp.y + 5, msg.info.height)):
                        if self.map.data[j * msg.info.width + i] == 0:
                            self.map.data[j * msg.info.width + i] = self.map.data[temp.y * msg.info.width + temp.x] + 1
                            queue.append(Pixel(i, j))

        pub_map = PotentialGrid()
        pub_map.header = self.map.header
        pub_map.info = self.map.info
        pub_map.data = tuple(self.map.data)
        self.pub.publish(pub_map)
        self.mutex.release()


class MissionHandler:
    """Class to provide the turtle specified waypoint."""

    def __init__( self ):
        """Class constructor."""
        rospy.init_node('mission_node')

        self.mutex = Lock()

        # Initialize flags
        self.fresh_frontiers = False
        
        # Initalize variables
        self.robot_x = self.robot_y = self.robot_theta = None

        self.frontiers = None
        self.waypoint_filter = WaypointHistory()

        # UPDATE
        self.found_waypoint = False
        self.hist_count = 1
        self.bumper_count = 1
        self.start_x = 0.0
        self.start_y = 0.0
        self.started = False
        self.returned = False
        self.startTime = rospy.get_time()
        self.target_goal = PoseStamped()
        self.current_wp = Pose2D()
        self.auto_goal = Pose2D()
        # init target_goal
        self.target_goal.header.seq = 0
        self.target_goal.header.stamp = rospy.get_rostime()
        self.target_goal.header.frame_id = "map"
        self.target_goal.pose.position.z = 0.0
        self.target_goal.pose.orientation.x = 0.0
        self.target_goal.pose.orientation.y = 0.0
        self.target_goal.pose.orientation.z = 0.0
        self.target_goal.pose.orientation.w = 1.0

        # Mission Handler Publisher
        self.pub_rviz = rospy.Publisher("/mission_visualize", Marker, queue_size=10)

        self.latencyPose = Pose2D()
        self.latency = rospy.Timer(rospy.Duration(15), self.latencyCallback)

        # Mission Handler Subscribers
        rospy.Subscriber("/odom", Odometry, self.odometryCallback)
        # rospy.Subscriber("/potential_map", PotentialGrid, self.frontierCallback, queue_size=1)

        # UPDATE
        rospy.Subscriber("/move_base_simple/auto_goal_find", Int8, self.autoGoalFindCallback, queue_size=1)
        rospy.Subscriber("/move_base_simple/invalid_path", Int8, self.invalidPathCallback, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.anotherGoalCallback, queue_size=1)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumperCallback, queue_size=1)

        # Add service
        rospy.wait_for_service('force_find')
        try:
            self.propose_ForceFind = rospy.ServiceProxy('force_find', ForceFind, persistent=True)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", str(e))
        
        self.auto_goal_pub_ = rospy.Publisher("/move_base_simple/auto_goal", PoseStamped, queue_size=1)
        self.control_input_pub_ = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 1)


    # rotate robot
    def rotateOnce(self):
        rospy.loginfo("current orientation:%s", str(self.robot_theta))
        control_input = Twist()
        control_input.angular.z = 0.5
        while (np.abs(self.robot_theta) < 0.5).any():
           self.control_input_pub_.publish(control_input)

        rotate = 0
        while True:
            self.control_input_pub_.publish(control_input)
            if (np.abs(self.robot_theta) < 0.1).any():
                rotate += 1
                rospy.loginfo("rotate finish")
            if rotate == 1:
                break

    def anotherGoalCallback(self, msg):
        rospy.logwarn("got another goal")
        self.target_goal = msg


    def invalidPathCallback(self, msg):
        self.mutex.acquire()

        self.hist_count += 1
        if self.returned is False:
            if self.hist_count < 3:
                # rospy.logwarn("got invalid, use past goals")
                # print(self.waypoint_filter.wp_history)
                # if (self.hist_count < 4) and self.waypoint_filter.wp_history is not None and self.waypoint_filter.wp_history.shape[0] > self.hist_count:
                    
                #     self.target_goal.pose.position.x = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 0]
                #     self.target_goal.pose.position.y = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 1]

                #     rospy.logwarn("got past goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                #     self.auto_goal_pub_.publish(self.target_goal)
                pass
            elif self.hist_count < 7:
                rospy.loginfo("got invalid, use rotate")
                control_input = Twist()
                control_input.angular.z = 1.57
                self.control_input_pub_.publish(control_input)

                req = ForceFindRequest()
                resGoal = ForceFindResponse()
                req.callFlag = 1
                try:
                    resGoal = self.propose_ForceFind(req)
                except rospy.ServiceException, e:
                    rospy.logwarn("Request error: %s", str(e))

                self.target_goal = resGoal.goal
                rospy.logwarn("Force Find: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                self.auto_goal_pub_.publish(self.target_goal)
            else:
                self.fresh_frontiers = False
                self.found_waypoint = False
                # self.returned = False
                rospy.logwarn("got invalid too many times, back safety distance")
                safety_dis = 0.20

                if self.hist_count < 10:
                    self.target_goal.pose.position.x = self.robot_x - (math.cos(self.robot_theta)*safety_dis + np.random.random()*0.1)
                    self.target_goal.pose.position.y = self.robot_y - (math.sin(self.robot_theta)*safety_dis + np.random.random()*0.1)
                else:
                    self.target_goal.pose.position.x = self.robot_x + (math.cos(self.robot_theta)*safety_dis + np.random.random()*0.1)
                    self.target_goal.pose.position.y = self.robot_y + (math.sin(self.robot_theta)*safety_dis + np.random.random()*0.1)

                rospy.logwarn("back goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                self.auto_goal_pub_.publish(self.target_goal)
        else:
            if self.hist_count > 4:
                rospy.logwarn("got invalid too many times, use rotate")
                control_input = Twist()
                control_input.angular.z = 1.57
                self.control_input_pub_.publish(control_input)

        self.mutex.release()


    def bumperCallback(self, msg):
        self.mutex.acquire()

        self.bumper_count += 1
        # init vars
        self.fresh_frontiers = False
        self.found_waypoint = False
        # self.returned = False

        if self.returned is False:
            if self.bumper_count < 3:
                req = ForceFindRequest()
                resGoal = ForceFindResponse()
                req.callFlag = 1
                try:
                    resGoal = self.propose_ForceFind(req)
                except rospy.ServiceException, e:
                    rospy.logwarn("Request error: %s", str(e))

                if resGoal.goal is not None:
                    self.target_goal = resGoal.goal
                    rospy.logwarn("Force Find: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                    self.auto_goal_pub_.publish(self.target_goal)
                    
            # elif (self.bumper_count < 5) and self.waypoint_filter.wp_history is not None and self.waypoint_filter.wp_history.shape[0] > (self.bumper_count - 4):
            #     rospy.logwarn("got bumper, use past goals")
            #     print(self.waypoint_filter.wp_history)
            #     self.target_goal.pose.position.x = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - (self.bumper_count - 4), 0]
            #     self.target_goal.pose.position.y = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - (self.bumper_count - 4), 1]

            #     rospy.logwarn("got past goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
            #     self.auto_goal_pub_.publish(self.target_goal)
            else:
                rospy.logwarn("got bumper too many times, back safety distance")
                safety_dis = 0.20
                self.target_goal.pose.position.x = self.robot_x - (math.cos(self.robot_theta)*safety_dis + np.random.random()*0.1)
                self.target_goal.pose.position.y = self.robot_y - (math.sin(self.robot_theta)*safety_dis + np.random.random()*0.1)

                rospy.logwarn("back goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                self.auto_goal_pub_.publish(self.target_goal)
        else:
            if self.bumper_count == 2:
                self.auto_goal.x = self.target_goal.pose.position.x
                self.auto_goal.y = self.target_goal.pose.position.y
            elif self.bumper_count < 6:
                rospy.logwarn("got bumper, use rotate")
                control_input = Twist()
                control_input.angular.z = 1.57
                self.control_input_pub_.publish(control_input)
            else:
                rospy.logwarn("got bumper too many times, back safety distance")
                safety_dis = 0.20

                self.target_goal.pose.position.x = self.robot_x - (math.cos(self.robot_theta)*safety_dis + np.random.random()*0.1)
                self.target_goal.pose.position.y = self.robot_y - (math.sin(self.robot_theta)*safety_dis + np.random.random()*0.1)

                rospy.logwarn("back goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                self.auto_goal_pub_.publish(self.target_goal)
        
        self.mutex.release()


    def autoGoalFindCallback(self, msg):
        # self.current_wp.x = self.target_goal.pose.position.x
        # self.current_wp.y = self.target_goal.pose.position.y
        # self.updateCurrentWaypoint()

        self.showTextRobot("exploring")
        # rospy.loginfo("started find, current goals:")
        # print(self.frontiers)

        # self.proposeWaypoints()

        if self.returned is True:
            if self.bumper_count > 2:
                rospy.logwarn("republish goal")
                self.target_goal.pose.position.x = self.auto_goal.x
                self.target_goal.pose.position.y = self.auto_goal.y
                self.bumper_count = 1
                self.auto_goal_pub_.publish(self.target_goal)
                return
            self.showTextRobot("exploring")
            rospy.logwarn("returned, auto navi ended")
            rospy.logwarn("please use two point navigation")
            return

        self.bumper_count = 1
        self.hist_count = 1

        # if self.found_waypoint:
        #     rospy.loginfo("found goal")
            
        #     self.target_goal.pose.position.x = self.auto_goal.x
        #     self.target_goal.pose.position.y = self.auto_goal.y

        #     rospy.loginfo("selected: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
        #     self.auto_goal_pub_.publish(self.target_goal)

        #     self.hist_count = 1
        #     self.returned = False
        
        req = ForceFindRequest()
        resGoal = ForceFindResponse()
        req.callFlag = 1
        if self.returned is False:
            try:
                resGoal = self.propose_ForceFind(req)
            except rospy.ServiceException, e:
                rospy.logwarn("Request error: %s", str(e))
        else:
            resGoal.goal.pose.position.x = 0.0
            resGoal.goal.pose.position.y = 0.0
        
        if resGoal.goal is not None:
            if abs(resGoal.goal.pose.position.x - 0.0) > 1e-6 or abs(resGoal.goal.pose.position.y - 0.0) > 1e-6:
                self.target_goal = resGoal.goal
                rospy.logwarn("Force Find: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                self.auto_goal_pub_.publish(self.target_goal)
                self.current_wp.x = self.target_goal.pose.position.x
                self.current_wp.y = self.target_goal.pose.position.y
            else:
                # if (rospy.get_time() - self.startTime) < 240.0:
                #     self.hist_count = 1
                #     rospy.logerr("too early, try again later, use random now!!!")
                #     rospy.logerr("too early, try again later, use random now!!!")
                #     rospy.logerr("too early, try again later, use random now!!!")
                #     self.target_goal.pose.position.x = self.robot_x + np.random.random()
                #     self.target_goal.pose.position.y = self.robot_y + np.random.random()
                #     rospy.logwarn("current goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                #     self.auto_goal_pub_.publish(self.target_goal)
                #     return
                rospy.logwarn("got return flag")
                if self.returned is False:
                    self.target_goal.pose.position.x = self.start_x
                    self.target_goal.pose.position.y = self.start_y

                    rospy.logwarn("back to start: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                    self.auto_goal_pub_.publish(self.target_goal)
                    self.showTextRobot("returning")
                    
                    self.returned = True
                    rospy.logwarn("return phase")
                

    def odometryCallback(self, msg):
        self.mutex.acquire()

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y 

        quaternion = (msg.pose.pose.orientation.x, 
                      msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, 
                      msg.pose.pose.orientation.w)

        self.robot_theta = euler_from_quaternion(quaternion)
        self.robot_theta = angle_wrap(self.robot_theta[2])

        if self.started is False:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.started = True

            rospy.loginfo("start point: %s %s", str(self.start_x), str(self.start_y))

        self.mutex.release()


    # Calling callback
    def frontierCallback(self, msg):
        self.mutex.acquire()

        rospy.loginfo("got potential_map, Start looking for the target points")

        w = int(msg.info.width)
        self.frontiers = None

        for i in range(2, msg.info.height - 1):
            for j in range(2, msg.info.width - 1):
                # If not free space, continue
                if msg.data[w*i + j] < 3:
                    continue

                if  (msg.data[w*i + j - 1] == 2 and msg.data[w*(i - 1) + j] == 1) or \
                        (msg.data[w*i + j - 1] == 1 and msg.data[w*(i - 1) + j] == 2) or \
                        (msg.data[w*i + j + 1] == 2 and msg.data[w*(i - 1) + j] == 1) or \
                        (msg.data[w*i + j + 1] == 1 and msg.data[w*(i - 1) + j] == 2) or \
                        (msg.data[w*(i + 1) + j] == 2 and msg.data[w*i + j + 1] == 1) or \
                        (msg.data[w*(i + 1) + j] == 1 and msg.data[w*i + j + 1] == 2) or \
                        (msg.data[w*(i + 1) + j] == 2 and msg.data[w*i + j - 1] == 1) or \
                        (msg.data[w*(i + 1) + j] == 1 and msg.data[w*i + j - 1] == 2) :
                        # (msg.data[w*(i + 1) + j] == 2 and msg.data[w*(i - 1) + j] == 1) or \
                        # (msg.data[w*(i + 1) + j] == 1 and msg.data[w*(i - 1) + j] == 2) or \
                        # (msg.data[w*i + j + 1] == 2 and msg.data[w*i + j - 1] == 1) or \
                        # (msg.data[w*i + j + 1] == 1 and msg.data[w*i + j - 1] == 2) :
                        # (msg.data[w*(i + 1) + j] == 2 and msg.data[w*(i - 1) + j] == 2) or \
                        # (msg.data[w*(i + 1) + j] == 1 and msg.data[w*(i - 1) + j] == 1) or \
                        # (msg.data[w*i + j + 1] == 2 and msg.data[w*i + j - 1] == 2) or \
                        # (msg.data[w*i + j + 1] == 1 and msg.data[w*i + j - 1] == 1) :

                    frontier_x = float(j * msg.info.resolution + msg.info.origin.position.x)
                    frontier_y = float(i * msg.info.resolution + msg.info.origin.position.y)

                    angle_to_robot = np.abs(angle_wrap(self.robot_theta - np.arctan2(frontier_y - self.robot_y, frontier_x - self.robot_x)))
                    frontier_weight = msg.data[w*i + j] * (1 + 0.8*np.exp(-2.0/angle_to_robot))

                    if self.frontiers is None:
                        self.frontiers = np.array([[frontier_x, frontier_y, frontier_weight]])
                    else:                    
                        self.frontiers = np.vstack((self.frontiers, np.array([[frontier_x, frontier_y, frontier_weight]])))

        rospy.loginfo("potential goals")
        print(self.frontiers)
        self.fresh_frontiers = True
        self.mutex.release()


    # Calling frontiers function
    def proposeWaypoints(self):
        if self.fresh_frontiers is not True:
            rospy.logerr("not got potential_map yet, please request later!")
            rospy.logerr("not got potential_map yet, please request later!")
            rospy.logerr("not got potential_map yet, please request later!")
            self.found_waypoint = False
            return

        self.mutex.acquire()

        self.found_waypoint = False
        
        if self.frontiers is None:
            if self.returned is False:
                rospy.logwarn("no suitable target point, use 0.0 for goal")
                self.frontiers = np.array([[0.1, 0.1, 1.0]])
            else:
                self.found_waypoint = False
                self.fresh_frontiers = True
                self.mutex.release()
                return
       
        row_min = np.argmin(self.frontiers[:, 2])

        if (self.auto_goal.x != self.frontiers[row_min, 0] or self.auto_goal.y != self.frontiers[row_min, 1]):
            self.auto_goal.x = self.frontiers[row_min, 0]
            self.auto_goal.y = self.frontiers[row_min, 1]

            self.current_wp.x = self.frontiers[row_min, 0]
            self.current_wp.y = self.frontiers[row_min, 1]
            self.found_waypoint = True
            
        else:
            rospy.logwarn("found same goal or not found")

        self.fresh_frontiers = False
        self.mutex.release()


    def updateCurrentWaypoint(self):
        if self.current_wp is not None:
            self.waypoint_filter.addWaypointToHistory(self.current_wp)


    def showTextRobot(self, text):
        marker = Marker()

        marker.header.frame_id = "/base_link"
        marker.frame_locked = True
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_text"
        marker.id = 0

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = marker.color.g = 0.9
        marker.color.b = 0.6

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5

        marker.text = text

        self.pub_rviz.publish(marker)


    def latencyCallback(self, evevt):
        if abs(self.latencyPose.x - self.robot_x) > 1e-6 or abs(self.latencyPose.y - self.robot_y) > 1e-6:
            self.latencyPose.x = self.robot_x
            self.latencyPose.y = self.robot_y
        elif self.returned is False:
            rospy.loginfo("running latencyCallback to rotate")
            control_input = Twist()
            control_input.angular.z = 1.57
            self.control_input_pub_.publish(control_input)


if __name__ == '__main__':

    mission_handler = MissionHandler()
    # map_maker = PotentialMapMaker()
    rospy.spin()
