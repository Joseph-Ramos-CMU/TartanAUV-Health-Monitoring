from enum import Enum

class AlarmLevel(Enum):
    """ Eunm for specificying the severity of particular alarms
    """
    PREDIVE=1
    NEUTRAL=2 # Should not cause the robot to taken any additional actions
    SURFACE=3
    ABORT=4