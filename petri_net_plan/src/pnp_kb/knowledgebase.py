from abstract_knowledgebase import AbstractKnowledgeBase
from threading import Lock


class KnowledgeBase(AbstractKnowledgeBase):
    def __init__(self, **kwargs):
        self.__dict__["lock"] = Lock()
        if kwargs is not None:
            for k, v in kwargs.iteritems():
                self.update(k, v)

    def query(self, variable):
        print "--- QUERY ---", variable, getattr(self, variable)

        return getattr(self, variable)

    def update(self, variable, value):
        setattr(self, variable, value)

    def __setattr__(self, name, value):
        with self.lock:
            self.__dict__["_mutex_"+name] = value

    def __getattr__(self, name):
        with self.lock:
            try:
                return self.__dict__["_mutex_"+name]
            except KeyError as e:
                return None

