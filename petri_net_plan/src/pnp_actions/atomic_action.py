from abc import ABCMeta, abstractmethod
from threading import Thread, Lock


class AtomicAction(object):
    __metaclass__ = ABCMeta

    OUTCOMES = ("succeeded", "preempted", "failed")
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES

    def __init__(self, name, params=None):
        self.name = name
        self.params = params if params is not None else {}
        self.__mutex__ = Lock()
        self.__event = None
        self.__monitor_thread = None

    def start(self, kb, external_kb, event):
        self.__event = event
        self.__t = Thread(target=self.run, args=(kb, external_kb))
        self.__t.start()

    @abstractmethod
    def run(self, kb ,external_kb):
        return

    def trigger_event(self):
        self.__t.join()
        self.__event.set()

    def monitor(self):
        if self.__monitor_thread is None or not self.__monitor_thread.is_alive():
            self.__monitor_thread = Thread(target=self.trigger_event)
            self.__monitor_thread.start()
        return self.__monitor_thread

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

