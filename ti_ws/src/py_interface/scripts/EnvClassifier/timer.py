import rospy
import functools
from datetime import datetime

def timer(func):
    """Print the runtime of the decorated function"""
    @functools.wraps(func)
    def wrapper_timer(*args, **kwargs):
        start_time = datetime.now()   # 1
        value = func(*args, **kwargs)
        end_time = datetime.now()      # 2
        run_time = end_time - start_time    # 3
        rospy.loginfo("{0} finished in {1} secs.".format(func.__name__, run_time.total_seconds()))
        return value
    return wrapper_timer
