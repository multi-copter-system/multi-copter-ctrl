from enum import Enum


class MultiCopterCmd(Enum):
    # For `control_center` service
    START = 'start'
    READY = 'ready'
    HALT = 'halt'
    ARMING = 'arming'
    BEGIN_MOVE = 'begin_move'
    BEGIN_RETURN = 'begin_return'

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
