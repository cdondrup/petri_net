from abc import ABCMeta, abstractmethod


class Action(object):
    __metaclass__ = ABCMeta

    OUTCOMES = ("succeeded", "preempted", "failed")
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES

    def __init__(self, name, params=None, recovery=None):
        self.name = name
        self.params = params if params is not None else {}
        if recovery is None:
            self.recovery = {o: None for o in self.OUTCOMES}
        else:
            self.recovery = recovery
            for o in set(self.OUTCOMES) - set(self.recovery.keys()):
                self.recovery[o] = None

    @abstractmethod
    def start(self, kb):
        return

    @abstractmethod
    def monitor(self, kb):
        return

    @property
    @abstractmethod
    def succeeded(self):
        return

    @property
    @abstractmethod
    def preempted(self):
        return

    @property
    @abstractmethod
    def failed(self):
        return

