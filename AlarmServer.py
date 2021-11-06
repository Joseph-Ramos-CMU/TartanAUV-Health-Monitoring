import time
import rospy
# Rospy tutorials: writing a simple publisher and subscriber
from std_msgs.msg import String, Float32

# Nodes can communicate over pub/sub
# Each node can havve timers, which work like multiple threads being run in one thread
class AlarmServer():
    def __init__(self, number):
        pass