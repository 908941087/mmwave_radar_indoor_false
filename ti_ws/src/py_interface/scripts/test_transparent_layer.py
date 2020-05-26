#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Polygon, PolygonStamped, Point32

def publish_obstacle_msg():
    rospy.init_node("test_obstacle_msg")

    pub = rospy.Publisher('/transparent_obstacle', Polygon, queue_size=1)

    obstacle_msg = Polygon()

    # Add polygon obstacle
    v1 = Point32()
    v1.x = -1
    v1.y = -1
    v2 = Point32()
    v2.x = -0.5
    v2.y = -1.5
    v3 = Point32()
    v3.x = 0
    v3.y = -1
    v4 = Point32()
    v4.x = 0
    v4.y = -1
    obstacle_msg.points = [v1, v2, v3, v4]

    r = rospy.Rate(10) # 10hz
    t = 0.0
    while not rospy.is_shutdown():

        # Vary y component of the point obstacle
        obstacle_msg.points[0].y = 1*math.sin(t)
        t = t + 0.1

        pub.publish(obstacle_msg)

        r.sleep()

if __name__ == '__main__':
    try:
        publish_obstacle_msg()
    except rospy.ROSInterruptException:
        pass