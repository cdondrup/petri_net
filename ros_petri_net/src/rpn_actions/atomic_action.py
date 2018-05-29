from abc import ABCMeta, abstractmethod


class AtomicAction(object):
    __metaclass__ = ABCMeta

    OUTCOMES = ("succeeded", "preempted", "failed")
    TESTS = ("preconditions_met", "preconditions_not_met", "effects_achieved", "effects_not_achieved")
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES
    PRECONDITIONS_MET, PRECONDITIONS_NOT_MET, EFFECTS_ACHIEVED, EFFECTS_NOT_ACHIEVED = TESTS

    def __init__(self, name, params=None, recovery=None, preconditions=None, effects=None):
        self.name = name
        self.params = params if params is not None else {}
        if recovery is None:
            self.recovery = {o: None for o in self.OUTCOMES}
        else:
            self.recovery = recovery
            for o in set(self.OUTCOMES) - set(self.recovery.keys()):
                self.recovery[o] = None
        self.preconditions = preconditions if preconditions is not None else {}
        self.effects = effects if effects is not None else {}

    @abstractmethod
    def start(self, kb):
        return

    @abstractmethod
    def monitor(self, kb):
        return

    @property
    @abstractmethod
    def succeeded(self):
        return True

    @property
    @abstractmethod
    def preempted(self):
        return True

    @property
    @abstractmethod
    def failed(self):
        return True

