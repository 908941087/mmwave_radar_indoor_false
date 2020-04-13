#!/usr/bin/env python

# TODO is it necessary here?
import roslib; roslib.load_manifest('autonomous_explore_map_plan')
import rospy

from driver import driver

if __name__ == '__main__':
    try:
        
        pilot = driver()
        pilot.load_goals()
        pilot.drive()
        
        
    except rospy.ROSInterruptException:
        pass
