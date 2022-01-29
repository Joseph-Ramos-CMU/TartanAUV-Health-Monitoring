# Nodes can communicate over pub/sub
# Each node can have timers, which work like multiple threads being run in one thread
from typing_extensions import ParamSpecKwargs
from typing import Tuple
from health_node.AlarmDefs import AlarmDefs
from health_node import AlarmLevel
from health_monitor.msg import ServerToClientAlarmMsg

class AlarmReport():
    """
    Representation of the status of all alarams

    Not thread safe to write to: should only be modified by one thread at a time

    Generally speaking, the only classes that should interact with this class directly
    are the AlarmClient and AlarmServer

    Represented internally as a bitstring/ array of ints, where each int
    is equivalent
    """
    # number of elements in individual elements of the array
    # ints in Python are 4 bytes
    ELEMENT_SIZE = 32
    def __init__(self, alarms : 'list[int]' = None):
        if alarms is None:
            alarms = [0] * (AlarmDefs.NUM_ALARMS // AlarmReport.ELEMENT_SIZE)
        else:
            if len(alarms) * AlarmReport.ELEMENT_SIZE != AlarmDefs.NUM_ALARMS:
                raise ValueError("Size of alarms is wrong")
        # alarms is interpreted as a bit string/ boolean array, but stored as an int array
        # for efficiency
        self.alarms = alarms
        self.numAlarms = len(alarms) * AlarmReport.ELEMENT_SIZE

    def __idToArrayIndex(self, id : int) -> Tuple[int, int]:
        """Returns the outer index and the inner index (specific bit)
        that the id is located at in the alarms list
        
        Keyword arguments:
        id -- the id number of the alarm: should be gotten from the AlarmDefs class
        """
        temp : int = id // self.ELEMENT_SIZE
        return (temp, id - temp)

    def checkStatus(self, id : int) -> bool:
        """Gets the value of the specified alarm
        
        Keyword arguments:
        id -- the id number of the alarm: should be gotten from the AlarmDefs class
        """
        if id < 0 or id >= self.numAlarms:
            raise ValueError("Invalid id")
        (index, subIndex) = self.__idToArrayIndex(id)

        value : int = (self.alarms[index] >> subIndex) & 0x1
        return value == 1

    def setStatus(self, id : int, value : bool) -> None:
        if id < 0 or id >= self.numAlarms:
            raise ValueError("Invalid id")
        (index, subIndex) = self.__idToArrayIndex(id)
        alarmInt : int = self.alarms[index]
        bitmask : int = 1 << subIndex
        if value:
            alarmInt = alarmInt | bitmask
        else:
            bitmask = ~bitmask
            alarmInt = alarmInt & bitmask
        self.alarms[index] = alarmInt

    def toIntList(self) -> 'list[int]':
        return self.alarms

    def getHighestAlarmLevel() -> AlarmLevel:
        """
        """
        pass

    def toMsg(self) -> ServerToClientAlarmMsg:
        """
        Returns a ROS msg representing this report
        """
        msg = ServerToClientAlarmMsg()
        msg.bitmask = self.alarms
        return msg

    def __str__(self) -> str:
        return str(self.alarms)

# Unfortunately, some pecularities of python type annotations don't allow me to annotate this with
# AlarmReport. I instead need to use a string 'AlarmReport'
def msgToAlarmReport(msg : ServerToClientAlarmMsg) -> 'AlarmReport':
    """
    Returns a ROS msg representing this report
    """
    return AlarmReport(msg.bitmask)

def main():
    # TODO: write unit tests
    testReport1 : AlarmReport = AlarmReport()
    assert testReport1.checkStatus(AlarmDefs.NUM_ALARMS - 1) is False
    testReport1.setStatus(AlarmDefs.NUM_ALARMS - 1, True)
    assert testReport1.checkStatus(AlarmDefs.NUM_ALARMS - 1) is True

    print ("All tests pasts")

if __name__ == '__main__':
    main()