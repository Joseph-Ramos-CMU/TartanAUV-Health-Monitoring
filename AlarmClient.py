import AlarmLevel
import rospy
# Rospy tutorials: writing a simple publisher and subscriber
from std_msgs.msg import String, Float32

# Nodes can communicate over pub/sub
# Each node can havve timers, which work like multiple threads being run in one thread
class AlarmClient():
    def __init__(self, number):
        # The cached version of the report
        self.report = None
    
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
        pass

    def clear(self, id : int) -> None:
        """Equivalent to set(self, id, False)"""
        set(self, id, False)

    def checkStatus(self, id : int) -> bool:
        """Gets the value of the specified alarm
        
        Keyword arguments:
        id -- the id number of the alarm: should be gotten from the AlarmDefs class
        """
        # Checks the cached alarm report: a different
        return self.report.checkStatus(id)

    def getHighestAlarmLevel() -> AlarmLevel:
        """
        """
        pass