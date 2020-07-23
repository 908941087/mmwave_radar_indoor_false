#!/usr/bin/env python
"""
scp pi@10.42.0.1:/home/pi/mmwave_radar_indoor_false/ti_ws/src/autonomous_nav/src/mission_node_only_BFS.py ./lab/radar/mmwave_radar_indoor_false/ti_ws/src/autonomous_nav/src/
scp ./lab/radar/mmwave_radar_indoor_false/ti_ws/src/autonomous_nav/src/mission_node_only_BFS.py pi@10.42.0.1:/home/pi/mmwave_radar_indoor_false/ti_ws/src/autonomous_nav/src/
"""

# ROS imports
import time

import roslib
import rospy
import numpy as np
import math
from enum import Enum
from threading import Lock, Condition
from collections import deque

#ROS messages
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

from autonomous_nav.srv import ForceFind, ForceFindRequest, ForceFindResponse


class MissionStatus(Enum):
    READY = 0
    RUNNING = 1
    WAITING = 2
    STOPPING = 3
    TERMINATED = 4


class GoalProductionStatus(Enum):
    READY = 0
    CALCULATING = 1


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


def getAngle(v1, v2):
    theta = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
    if theta < -np.pi:
        theta += 2 * np.pi
    elif theta > np.pi:
        theta -= 2 * np.pi
    return theta


class MissionHandler:
    """Class to maintain the robot state and instruct robot to:
    1. navigate through the map
    2. avoid and deal with collision
    3. avoid and deal with other unspecified situations
    """

    def __init__(self, goal_keeper, goal_producer):
        """Class constructor."""
        rospy.init_node('mission_node')

        self.mutex = Lock()
        self.pos_mutex = Lock()
        self.update_robot_pos = True

        # Initalize variables
        self.robot_x = self.robot_y = self.robot_theta = None

        # mission_status
        self.invalid_path_count = 0
        self.collision_count = 0
        self.return_count = 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_time = rospy.get_time()
        self.goal_keeper = goal_keeper
        self.goal_producer = goal_producer
        self.mission_status = MissionStatus.READY
        self.goal_status = GoalProductionStatus.READY

        # init target_goal
        self.target_goal = PoseStamped()
        self.target_goal.header.seq = 0
        self.target_goal.header.stamp = rospy.get_rostime()
        self.target_goal.header.frame_id = "map"
        self.target_goal.pose.position.z = 0.0
        self.target_goal.pose.orientation.x = 0.0
        self.target_goal.pose.orientation.y = 0.0
        self.target_goal.pose.orientation.z = 0.0
        self.target_goal.pose.orientation.w = 1.0

        self.recent_invalid_goals = deque()
        self.goal_queue = deque()  # record all the goals

        # Mission Handler Publisher
        self.pub_rviz = rospy.Publisher("/mission_visualize", Marker, queue_size=10)

        # Mission Handler Subscribers
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometryCallback)
        self.auto_goal_sub = rospy.Subscriber("/move_base_simple/auto_goal_find", Int8, self.autoGoalFindCallback, queue_size=1)
        self.invalid_path_sub = rospy.Subscriber("/move_base_simple/invalid_path", Int8, self.invalidPathCallback, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.newGoalCallback, queue_size=1)
        self.bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumperCallback, queue_size=1)

        # Publishers
        self.auto_goal_pub_ = rospy.Publisher("/move_base_simple/auto_goal", PoseStamped, queue_size=1)
        self.control_input_pub_ = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 1)

    def newGoalCallback(self, msg):
        """Get new goal from other sources."""
        rospy.logwarn("Got a goal from outside.")
        self.target_goal = msg
        rospy.logwarn("Outside goal orientation: ({0}, {1}, {2}, {3})".format(self.target_goal.pose.orientation.x,
                                                                              self.target_goal.pose.orientation.y,
                                                                              self.target_goal.pose.orientation.z,
                                                                              self.target_goal.pose.orientation.w))
        self.mission_status = MissionStatus.RUNNING

    def invalidPathCallback(self, msg):
        if MissionStatus.READY.value < self.mission_status.value:
            self.mutex.acquire()
            self.invalid_path_count += 1
            rospy.logwarn("Invalid path count: {0}.".format(self.invalid_path_count))
            if self.invalid_path_count < 3:
                self.mutex.release()
            elif self.invalid_path_count < 5:
                self.mutex.release()
                if self.isGoalInLaserRange(self.target_goal):
                    rospy.logwarn("Could not find a valid path but goal is in laser range. "
                                  "Going to new goal.")
                    self.goWhereRobotIs()
            elif self.invalid_path_count < 7:
                self.mutex.release()
                rospy.logwarn("Could not find a valid path.")
                rospy.logwarn("Rotate 180 degrees.")
                control_input = Twist()
                control_input.angular.z = 1.57
                self.control_input_pub_.publish(control_input)
                if not self.isGoalTooCloseToStart(self.target_goal):
                    self.getInvalidGoal(self.target_goal)
                if len(self.goal_keeper) != 0:  # already preloaded goal
                    self.goToNewGoal()
                else:
                    if len(self.goal_queue) != 0:
                        rospy.logwarn("Using old goal.")
                        self.goToNewGoal(self.goal_queue.pop())
                        self.invalid_path_count = 0
                    else:
                        self.goToNewGoal()
            else:
                rospy.logwarn("Robot is stuck. Trying to go back to start.")
                self.target_goal.pose.position.x = self.start_x
                self.target_goal.pose.position.y = self.start_x
                self.invalid_path_count = 0
                self.mutex.release()
                self.auto_goal_pub_.publish(self.target_goal)
        else:
            rospy.logwarn("Invalid call invalid path callback.")

    def bumperCallback(self, msg):
        if MissionStatus.READY.value < self.mission_status.value:
            self.mutex.acquire()
            self.collision_count += 1
            if not self.isGoalTooCloseToStart(self.target_goal):
                self.getInvalidGoal(self.target_goal)
            if self.collision_count < 4:
                self.mutex.release()
                rospy.logwarn("Encountered several collisions, trying with new goal.")
                rospy.logwarn("Going to new goal.")
                if len(self.goal_queue) != 0:
                    rospy.logwarn("Using old goal.")
                    self.goToNewGoal(self.goal_queue.pop())
                    self.collision_count = 0
                else:
                    self.goToNewGoal()
            else:
                rospy.logwarn("Encountered lots of collisions, back safety distance")
                safety_dis = 0.20
                self.update_robot_pos = True
                while self.update_robot_pos:
                    time.sleep(0.5)
                self.pos_mutex.acquire()
                self.target_goal.pose.position.x = self.robot_x - (math.cos(self.robot_theta)*safety_dis + np.random.random()*0.1)
                self.target_goal.pose.position.y = self.robot_y - (math.sin(self.robot_theta)*safety_dis + np.random.random()*0.1)
                self.pos_mutex.release()
                rospy.logwarn("Backing.")
                self.collision_count = 0
                self.mutex.release()  # use of robot_x robot_y needed lock
                self.goToNewGoal(self.target_goal)
        else:
            rospy.logwarn("Invalid call of bumper callback.")

    def autoGoalFindCallback(self, msg):
        """
        Called when received "Goal reached" msg.
        Generate new goal to go.
        :param msg:
        :return:
        """
        if self.mission_status == MissionStatus.READY:
            self.mission_status = MissionStatus.RUNNING
            rospy.logwarn("Mission Status Changed to RUNNING.")
        if MissionStatus.READY.value < self.mission_status.value < MissionStatus.STOPPING.value:
            rospy.logwarn("Previous goal reached. Going to new goal.")
            self.showTextRobot("exploring")
            self.mutex.acquire()
            self.collision_count = 0
            self.invalid_path_count = 0
            self.mutex.release()
            self.goToNewGoal()
        else:
            rospy.logwarn("Invalid call of auto goal find callback.")
            pass

    def odometryCallback(self, msg):
        if self.update_robot_pos:
            self.pos_mutex.acquire()
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_theta = euler_from_quaternion(quaternion)[2]
            if self.mission_status == MissionStatus.READY:
                self.start_x = self.robot_x
                self.start_y = self.robot_y
            self.pos_mutex.release()
            self.update_robot_pos = False
        else:
            pass

    def preloadGoal(self):
        """
        Preload the goal before current goal is reached to speed up the whole process.
        :return:
        """
        if len(self.goal_keeper) == 0 and self.mission_status == MissionStatus.RUNNING:  # RUNNING or WAITING
            rospy.logwarn("Preloading Goal...")
            res_goal = self.getNewGoal()
            if res_goal is not None:
                rospy.logwarn("Preloaded Goal: %s %s",
                              str(res_goal.pose.position.x),
                              str(res_goal.pose.position.y))
        else:
            rospy.logwarn("Preloading conditions not satisfied. Skipping.")

    def endNavigation(self):
        """
        End autonomous navigation, terminate all timed missions and auto navigation.
        :return:
        """
        self.showTextRobot("exploring")
        self.auto_goal_sub.unregister()
        rospy.logwarn("Auto goal subscription unregistered .")
        self.mission_status = MissionStatus.TERMINATED
        rospy.logwarn("Mission Status Changed to TERMINATED.")
        rospy.logwarn("Auto Navigation Ended.")
        rospy.logwarn("Please use two point navigation.")

    def getNewGoal(self):
        """
        Get a new goal using ForceFind. Guarantee that new goal is not too close to current goal.
        :return:
        """
        res_goal = self.goal_producer.produce()
        if res_goal is None:
            return None
        invalid_goal_count = 0
        while self.isGoalTooCloseToCurGoal(res_goal) or self.isCloseToInvalidGoals(res_goal):
            invalid_goal_count += 1
            if invalid_goal_count >= 3:
                rospy.logwarn("Got invalid goal too many times. Going back to start.")
                self.target_goal.pose.position.x = self.start_x
                self.target_goal.pose.position.y = self.start_y
                res_goal = self.target_goal
                break
            else:
                self.goal_keeper.popleft()
                res_goal = self.goal_producer.produce()
                if res_goal is None:
                    return None
        self.update_robot_pos = True
        while self.update_robot_pos:
            time.sleep(0.5)
        self.pos_mutex.acquire()
        direction = (res_goal.pose.position.x - self.robot_x, res_goal.pose.position.y - self.robot_y)
        self.pos_mutex.release()
        x_direction = (1, 0)
        theta = getAngle(direction, x_direction)
        q = quaternion_from_euler(0.0, -0.0, theta)
        res_goal.pose.orientation.x = q[0]
        res_goal.pose.orientation.y = q[1]
        res_goal.pose.orientation.z = q[2]
        res_goal.pose.orientation.w = q[3]
        rospy.logwarn("Theta: {0}.".format(theta))
        return res_goal

    def goToNewGoal(self, new_goal=None):
        """
        Find a new goal and go.
        :return:
        """
        if self.mission_status == MissionStatus.WAITING:  # already calculating for new goal, no need to calculate again.
            rospy.logwarn("Mission status WAITING. Skipping.")
            return
        if self.mission_status == MissionStatus.RUNNING:
            if new_goal is not None:  # use caller specified goal
                rospy.logwarn("Using specified goal.")
                res_goal = new_goal
            else:  # use calculated goal
                if not self.isGoalTooCloseToStart(self.target_goal):  # avoid going to start twice using this mechanism
                    self.goal_queue.append(self.target_goal)  # if the new goal is caller specified, then this function is not triggered by goal reached.
                if len(self.goal_keeper) == 0:
                    self.mission_status = MissionStatus.WAITING
                    rospy.logwarn("Mission Status Changed to WAITING.")
                    time.sleep(5.0)  # wait for map to update
                    self.getNewGoal()
                    while len(self.goal_keeper) == 0:
                        time.sleep(1.0)
                else:
                    rospy.logwarn("Using Preloaded Goal.")
                res_goal = self.goal_keeper.popleft()
            if res_goal is not None:
                if not self.isGoalTooCloseToStart(res_goal):
                    self.target_goal = res_goal
                    rospy.logwarn("Going to: %s %s", str(self.target_goal.pose.position.x),
                                  str(self.target_goal.pose.position.y))
                    rospy.logwarn("Orientation: ({0}, {1}, {2}, {3}).".format(self.target_goal.pose.orientation.x,
                                                                              self.target_goal.pose.orientation.y,
                                                                              self.target_goal.pose.orientation.z,
                                                                              self.target_goal.pose.orientation.w))
                    self.auto_goal_pub_.publish(self.target_goal)
                    self.mission_status = MissionStatus.RUNNING
                    rospy.logwarn("Mission Status Changed to RUNNING.")
                    self.return_count = 0
                    # wait for map to update
                    time.sleep(12.0)
                    self.preloadGoal()
                else:
                    self.return_count += 1
                    if MissionStatus.READY.value < self.mission_status.value < MissionStatus.STOPPING.value:
                        rospy.logwarn("Going back to start.")
                        self.target_goal.pose.position.x = self.start_x
                        self.target_goal.pose.position.y = self.start_y
                        self.auto_goal_pub_.publish(self.target_goal)
                        self.mission_status = MissionStatus.RUNNING
                        rospy.logwarn("Mission Status Changed to RUNNING.")
                        self.showTextRobot("returning")
                        if self.return_count >= 3:
                            self.mission_status = MissionStatus.STOPPING
                            rospy.logwarn("Mission Status Changed to STOPPING.")
                            self.endNavigation()
                        else:
                            self.preloadGoal()

    def isGoalTooCloseToCurGoal(self, goal):
        """
        Check if goal is too close to current goal. Eliminate the case of goal being (0.0, 0.0).
        :param goal:
        :return:
        """
        x = goal.pose.position.x
        y = goal.pose.position.y
        res = (not self.isGoalTooCloseToStart(goal)) and abs(self.target_goal.pose.position.x - x) < 0.3 and \
            abs(self.target_goal.pose.position.y - y) < 0.3
        if res:
            rospy.logwarn("New Goal ({0}, {1})is too close to current goal.".format(x, y))
        return res

    def isGoalTooCloseToStart(self, goal):
        """
        Check if goal is too close to start point, normally is (0, 0).
        :param goal:
        :return:
        """
        return abs(goal.pose.position.x - self.start_x) < 1e-6 and \
               abs(goal.pose.position.y - self.start_y) < 1e-6

    def getInvalidGoal(self, goal):
        if len(self.recent_invalid_goals) > 2:
            self.recent_invalid_goals.popleft()
        self.recent_invalid_goals.append(goal)

    def isCloseToInvalidGoals(self, goal):
        if self.isGoalTooCloseToStart(goal):
            return False
        for g in self.recent_invalid_goals:
            if self.goalDist(goal, g) < 1.0:
                rospy.logwarn("New Goal ({0}, {1}) is too close to invalid goals.".format(goal.pose.position.x,
                                                                                          goal.pose.position.y))
                return True
        return False

    def isGoalInLaserRange(self, goal):
        goal_x, goal_y = goal.pose.position.x, goal.pose.position.y
        self.update_robot_pos = True
        while self.update_robot_pos:
            time.sleep(0.5)
        self.pos_mutex.acquire()
        res = math.sqrt((goal_x - self.robot_x) ** 2 + (goal_y - self.robot_y) ** 2) < 5.0
        self.pos_mutex.release()
        return res

    def goWhereRobotIs(self):
        self.update_robot_pos = True
        while self.update_robot_pos:
            time.sleep(0.5)
        self.pos_mutex.acquire()
        self.target_goal.pose.position.x = self.robot_x
        self.target_goal.pose.position.y = self.robot_y
        self.pos_mutex.release()
        self.goToNewGoal(self.target_goal)

    @staticmethod
    def goalDist(g1, g2):
        """
        Calculate the distance of two goals of type PoseStamped()
        :param g1: PoseStamped(), goal 1
        :param g2: PoseStamped(), goal 2
        :return: float, distance between goal1 and goal2
        """
        x1, y1 = g1.pose.position.x, g1.pose.position.y
        x2, y2 = g2.pose.position.x, g2.pose.position.y
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

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


class GoalProducer:

    def __init__(self, goal_keeper):
        self.status = GoalProductionStatus.READY
        self.goal_keeper = goal_keeper

        # Add service
        rospy.wait_for_service('force_find')
        try:
            self.propose_ForceFind = rospy.ServiceProxy('force_find', ForceFind, persistent=True)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", str(e))

    def produce(self):
        if self.status == GoalProductionStatus.READY:
            req = ForceFindRequest()
            res_goal = ForceFindResponse()
            req.callFlag = 1
            try:
                rospy.logwarn("Requesting for new goal.")
                self.status = GoalProductionStatus.CALCULATING
                res_goal = self.propose_ForceFind(req)
                self.status = GoalProductionStatus.READY
            except rospy.ServiceException, e:
                rospy.logwarn("Request error: %s", str(e))
            finally:
                if len(self.goal_keeper) != 0:  # already have a goal in pocket
                    self.goal_keeper.popleft()
                self.goal_keeper.append(res_goal.goal)
                return res_goal.goal  # type of PoseStamped
        else:  # CALCULATING
            rospy.logwarn("Already Calculating Goal.")
            return None


if __name__ == '__main__':
    goal_keeper = deque()

    goal_producer = GoalProducer(goal_keeper)
    mission_handler = MissionHandler(goal_keeper, goal_producer)
    rospy.spin()
