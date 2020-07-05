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

        # UPDATE
        self.found_waypoint = False
        self.hist_count = 1
        self.bumper_count = 1
        self.start_x = 0.0
        self.start_y = 0.0
        self.started = False
        self.return_count = 0
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
        self.latency = rospy.Timer(rospy.Duration(10), self.latencyCallback)

        self.refind = rospy.Timer(rospy.Duration(300), self.refindCallback)

        # self.backToStart = rospy.Timer(rospy.Duration(30), self.backToStartCallback)

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
        # if self.returned is False:
        #     backToStart = rospy.Timer(rospy.Duration(160), self.backToStartCallback)
        self.target_goal = msg


    def invalidPathCallback(self, msg):
        self.mutex.acquire()

        self.hist_count += 1
        if self.returned is False:

            if self.hist_count > 3:
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

                if resGoal.goal is not None:
                    if (resGoal.goal.pose.position.x != 0.0 or resGoal.goal.pose.position.y != 0.0) and \
                            abs(self.target_goal.pose.position.x - resGoal.goal.pose.position.x) < 1e-6 and \
                            abs(self.target_goal.pose.position.y - resGoal.goal.pose.position.y) < 1e-6:
                        rospy.logwarn("Goal too close, pass Goal too close, pass Goal too close, pass")
                        return
                    self.target_goal = resGoal.goal
                    rospy.logwarn("Force Find: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                    self.auto_goal_pub_.publish(self.target_goal)

            # if self.hist_count < 3:
            #     # rospy.logwarn("got invalid, use past goals")
            #     # print(self.waypoint_filter.wp_history)
            #     # if (self.hist_count < 4) and self.waypoint_filter.wp_history is not None and self.waypoint_filter.wp_history.shape[0] > self.hist_count:
                    
            #     #     self.target_goal.pose.position.x = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 0]
            #     #     self.target_goal.pose.position.y = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 1]

            #     #     rospy.logwarn("got past goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
            #     #     self.auto_goal_pub_.publish(self.target_goal)
            #     pass
            # elif self.hist_count < 7:
            #     rospy.loginfo("got invalid, use rotate")
            #     control_input = Twist()
            #     control_input.angular.z = 1.57
            #     self.control_input_pub_.publish(control_input)

            #     req = ForceFindRequest()
            #     resGoal = ForceFindResponse()
            #     req.callFlag = 1
            #     try:
            #         resGoal = self.propose_ForceFind(req)
            #     except rospy.ServiceException, e:
            #         rospy.logwarn("Request error: %s", str(e))

            #     self.target_goal = resGoal.goal
            #     rospy.logwarn("Force Find: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
            #     self.auto_goal_pub_.publish(self.target_goal)
            # else:
            #     self.fresh_frontiers = False
            #     self.found_waypoint = False
            #     # self.returned = False

            #     control_input = Twist()
            #     control_input.angular.z = 1.57
            #     self.control_input_pub_.publish(control_input)

            #     # rospy.logwarn("got invalid too many times, back safety distance")
            #     # safety_dis = 0.20

            #     # if self.hist_count < 10:
            #     #     self.target_goal.pose.position.x = self.robot_x - (math.cos(self.robot_theta)*safety_dis + np.random.random()*0.1)
            #     #     self.target_goal.pose.position.y = self.robot_y - (math.sin(self.robot_theta)*safety_dis + np.random.random()*0.1)
            #     # else:
            #     #     self.target_goal.pose.position.x = self.robot_x + (math.cos(self.robot_theta)*safety_dis + np.random.random()*0.1)
            #     #     self.target_goal.pose.position.y = self.robot_y + (math.sin(self.robot_theta)*safety_dis + np.random.random()*0.1)

            #     # rospy.logwarn("back goal: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
            #     # self.auto_goal_pub_.publish(self.target_goal)
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
        # self.fresh_frontiers = False
        # self.found_waypoint = False
        # self.returned = False

        if self.returned is False:
            self.fresh_frontiers = True
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
                self.fresh_frontiers = True
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
            self.fresh_frontiers = True
            if abs(resGoal.goal.pose.position.x - 0.0) > 1e-6 or abs(resGoal.goal.pose.position.y - 0.0) > 1e-6:
                self.target_goal = resGoal.goal
                rospy.logwarn("Force Find: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                self.auto_goal_pub_.publish(self.target_goal)
                self.current_wp.x = self.target_goal.pose.position.x
                self.current_wp.y = self.target_goal.pose.position.y
                self.return_count = 0
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
                self.return_count += 1
                if self.returned is False:
                    self.target_goal.pose.position.x = self.start_x
                    self.target_goal.pose.position.y = self.start_y

                    rospy.logwarn("back to start: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
                    self.auto_goal_pub_.publish(self.target_goal)
                    self.showTextRobot("returning")

                    if self.return_count >= 2:
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

    def refindCallback(self, evevt):
        if self.returned is False:
            if self.fresh_frontiers is True:
                rospy.logwarn("refindCallback refindCallback refindCallback")
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
                    self.fresh_frontiers = False


    def backToStartCallback(self, evevt):
        if self.returned is False:
            self.target_goal.pose.position.x = self.start_x
            self.target_goal.pose.position.y = self.start_y

            rospy.logwarn("back to start: %s %s", str(self.target_goal.pose.position.x), str(self.target_goal.pose.position.y))
            self.auto_goal_pub_.publish(self.target_goal)
            self.showTextRobot("returning")
            
            self.returned = True
            rospy.logwarn("return phase")


if __name__ == '__main__':

    mission_handler = MissionHandler()
    rospy.spin()
