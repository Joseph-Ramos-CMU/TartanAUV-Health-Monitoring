from re import A
import time
import rospy
# Rospy tutorials: writing a simple publisher and subscriber
from std_msgs.msg import String, Float32
from threading import Lock
from health_node.AlarmReport import AlarmReport
from health_node.AlarmDefs import AlarmDefs
from health_monitor.msg import ServerToClientAlarmMsg


# http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber

# Nodes can communicate over pub/sub
# I think I don't need to explicitly ever create new threads for the server: just do some locks in the
# ROS supported callback functions/ timers
# Do I need to regularly publish new alarms? E.x. every 0.1 seconds?
class AlarmServer():
    def __init__(self):
        self.publisher = rospy.Publisher(AlarmDefs.SERVER_TO_CLIENT_PUB_SUB_ID, ServerToClientAlarmMsg, queue_size=10)
        self.report : AlarmReport = AlarmReport() # A new, empty alarm report

    def start(self):
        rospy.Timer(rospy.Duration(1.0), self.publishReport)

    def publishReport(self, timer_event) -> None:
        self.publisher.publish(self.report.toMsg())
        currentValue : bool = self.report.checkStatus(0)
        self.report.setStatus(0, not currentValue)
        print("MYNOTE2: AlarmServer: checkStatus: " + str( self.report.checkStatus(0)))

    def main():
        rospy.init_node(AlarmDefs.ALARM_SERVER_NODE_NAME)
        server = AlarmServer()
        server.start()
        print("AlarmServer started")
        rospy.spin() # Starts the timers: never returns: it will just infinitely
        # loop and run the self.go function we passed to the timer
        print("Error in AlarmServer")