from enum import Enum


class MultiCopterCmd(Enum):
    # For `control_center` service
    START = 'start'
    READY = 'ready'
    HALT = 'halt'
    ARMING = 'arming'
    DISARMING = 'disarming'
    BEGIN_MOVE = 'begin_move'
    FINISH_MOVE = 'finish_move'
    BEGIN_RETURN = 'begin_return'
    FINISH_RETURN = 'finish_return'

    # For `waypoint_manager` service
    READ = 'read'
    WRITE = 'write'

    @classmethod
    def is_member(cls, value):
        try:
            cls[value.upper()]
        except KeyError:
            return False
        else:
            return True
