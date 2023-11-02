from enum import Enum


class MultiCopterCmd(Enum):
    START = "start"
    READY = "ready"
    HALT = "halt"
    ARMING = "arming"
    DISARMING = "disarming"
    BEGIN_MOVE = "begin_move"
    FINISH_MOVE = "finish_move"
    BEGIN_RETURN = "begin_return"
    FINISH_RETURN = "finish_return"

    @classmethod
    def start(cls):
        return MultiCopterCmd.START.value

    @classmethod
    def ready(cls):
        return MultiCopterCmd.READY.value

    @classmethod
    def halt(cls):
        return MultiCopterCmd.HALT.value

    @classmethod
    def arming(cls):
        return MultiCopterCmd.ARMING.value

    @classmethod
    def disarming(cls):
        return MultiCopterCmd.DISARMING.value

    @classmethod
    def begin_move(cls):
        return MultiCopterCmd.BEGIN_MOVE.value

    @classmethod
    def finish_move(cls):
        return MultiCopterCmd.FINISH_MOVE.value

    @classmethod
    def begin_return(cls):
        return MultiCopterCmd.BEGIN_RETURN.value

    @classmethod
    def finish_return(cls):
        return MultiCopterCmd.FINISH_RETURN.value
