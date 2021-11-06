# Nodes can communicate over pub/sub
# Each node can havve timers, which work like multiple threads being run in one thread
from typing_extensions import ParamSpecKwargs

from health.AlarmDefs import AlarmDefs

"""
Represnetation of the status of all alarams

Represented internally as a bitstring/ array of ints, where each int
is equivalent
"""

class AlarmReport():
    # number of elements in individual elements of the array
    # ints in Python are 4 bytes
    ELEMENT_SIZE = 32
    def __init__(self, alarms : 'list[int]' = None):
        if alarms is None:
            alarms = [0] * (AlarmDefs.NUM_ALARMS / AlarmReport.ELEMENT_SIZE)
        else:
            if len(alarms) * AlarmReport.ELEMENT_SIZE != AlarmDefs.NUM_ALARMS:
                raise ValueError("Size of alarms is wrong")
        # alarms is interpreted as a bistring/ boolean array, but stored as an int array
        # for efficiency
        self.alarms = alarms
        self.numAlarms = len(alarms) * AlarmReport.ELEMENT_SIZE

    def checkStatus(self, id : int) -> bool:
        """Gets the value of the specified alarm
        
        Keyword arguments:
        id -- the id number of the alarm: should be gotten from the AlarmDefs class
        """
        if :

    
        

