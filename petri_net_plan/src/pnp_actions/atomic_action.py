from abc import ABCMeta, abstractmethod


class AtomicAction(object):
    __metaclass__ = ABCMeta

    OUTCOMES = ("succeeded", "preempted", "failed")
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES

    def __init__(self, name, params=None):
        self.name = name
        self.params = params if params is not None else {}

    @abstractmethod
    def start(self, kb ,external_kb):
        return

    @abstractmethod
    def monitor(self, kb, external_kb):
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

