from health_node  import AlarmLevel
from health_node import AlarmReport
from health_node import AlarmDefs
import rospy
# Rospy tutorials: writing a simple publisher and subscriber
from std_msgs.msg import String, Float32, Int32
from threading import Thread, Lock
from health_monitor.msg import ServerToClientAlarmMsg


"""
Whitelists - set which alarm to ignore
    Either means that the server and client alarm reports simply ignore these: the servers always leave
    them as false?
    Or have it as a different bitmask sent along with the report mask, then have the client
    ignore whitelisted exceptions: doing on client side makes synchronization issues easier
    
    Add this as a different option from no failure in the alarm levels: lowest severity

Look into ROS paramater for how to initialize the whitelist
    e.x. /health/whitelist, similar naming to how subscribers work
    Set in a launch file: a global paramater: doing it this way would make it so the server
    wouldn't have to manaully send it out
Launch file: says which nodes for ROS to run: can also set paramaters: we would have something
    like this for our whitelist (so we could have different versions for simulation and real world)

Server to client: uses pub sub
Client to server: service calls: in order to lock stuff

Look at ROSPY tutorial for service/ service calls: create a .srv service files
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv 
http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29 
Discussion of Service vs PubSub:
https://answers.ros.org/question/11834/when-should-i-use-topics-vs-services-vs-actionlib-actions-vs-dynamic_reconfigure/

I may be able to user ROS TImers instead of python threads: write a lot of invariants
Testing with multithreading: have them spam messages and see if errors occur
"""

# Nodes can communicate over pub/sub
# Each node can havve timers, which work like multiple threads being run in one thread
class AlarmClient():
    """
    Should be instantiated in every ROSS node that wishes to make use of alarms

    Manages keeping track of that node's internal Alarm state, and also communicates with the
    AlarmServer to update or recieve updates from the server as necessary

    Any updates to the AlarmReport should be done through this class, instead of interacting with
    the AlarmReport directly
    """

    def __init__(self):
        # The cached version of the report
        self.report : AlarmReport = None
        self.reportLock  : Lock = Lock()
        self.serverSubscriber = rospy.Subscriber(AlarmDefs.SERVER_TO_CLIENT_PUB_SUB_ID, ServerToClientAlarmMsg, self.__server_to_client_sub_callback)
        # self.thread = 


    def __server_to_client_sub_callback(self, msg : ServerToClientAlarmMsg) -> None:
        recievedReport = AlarmReport.msgToAlarmReport(msg)
        print("MYNOTE1: recievedReport: " + str(recievedReport))

    def set(self, id : int) -> None:
        """Equivalent to set(self, id, True)"""
        set(self, id, True)

    def set(self, id : int, value : bool) -> None:
        """Sends a message to the alarm server to set the status
        of an alarm based off of value

        Keyword arguments:
        id -- the id number of the alarm: should be gotten from the AlarmDefs class
        value -- what value the alarm should be set to
        """
        # I need to do stuff with modifying the cached report then send it to the server
        # while locking out the subscriber from changing the cache
        # Likely, I need a separate thread to handle most of my sets: I need to add a service call
        try:
            self.reportLock.acquire()
            result : bool = self.report.set(id, value)
        finally:
            self.reportLock.release()
        # TODO add the service call

    def clear(self, id : int) -> None:
        """Equivalent to set(self, id, False)"""
        set(self, id, False)

    def checkStatus(self, id : int) -> bool:
        """Gets the value of the specified alarm
        
        Keyword arguments:
        id -- the id number of the alarm: should be gotten from the AlarmDefs class
        """
        # Checks the cached alarm report: a different
        # Probably need to put a mutex around this
        try:
            self.reportLock.acquire()
            result : bool = self.report.checkStatus(id)
        finally:
            self.reportLock.release()
        return result
             
    def getHighestAlarmLevel() -> AlarmLevel:
        """
        """
        pass

    def communicateWithServer(self) -> None:
        """
        Ran in a separate thread
        """
        # https://stackoverflow.com/questions/3310049/proper-use-of-mutexes-in-python
        while True:
            pass