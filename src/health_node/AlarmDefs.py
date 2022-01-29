from health_node import AlarmLevel

class AlarmDefs():
    """
    Used to store static constants specifying properties of each alarm
    We cannot used shared object instances because different ROS nodes are essentially
    different processes
    This class should not be instantiated
    """ 

    def __init__(self):
        pass

    # The total number of alarms for the sub: should be a multiple of 32
    NUM_ALARMS : int = 32
    SERVER_TO_CLIENT_PUB_SUB_ID="alarm_server_to_client"
    ALARM_SERVER_NODE_NAME='alarm_server'

    """
    Each Alarm should have its own constants in the following form:
    Users of alarms will use these constants to get values like the alarm number:
    ID: the unique number for a particular alarm: specifies which bit in the alarm
        report corresponds to that number
    INIT: initial boolean value of the alarm
    ALARM_LEVEL: the severity of the alarm
    """
    # Comment describing the alarm
    ALARM_ZERO_ID=0
    ALARM_ZERO_INIT=False
    ALARM_ZERO_ALARM_LEVEL=AlarmLevel.PREDIVE

    # Comment describing the alarm
    ALARM_ONE_ID=1
    ALARM_ONE_INIT=False
    ALARM_ONE_ALARM_LEVEL=AlarmLevel.PREDIVE

    # Comment describing the alarm
    ALARM_TWO_ID=2
    ALARM_TWO_INIT=False
    ALARM_TWO_ALARM_LEVEL=AlarmLevel.PREDIVE