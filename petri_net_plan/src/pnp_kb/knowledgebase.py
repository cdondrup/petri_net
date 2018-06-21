from abstract_knowledgebase import AbstractKnowledgeBase
from threading import Lock


class KnowledgeBase(AbstractKnowledgeBase):
    def __init__(self, **kwargs):
        self.__dict__["lock"] = Lock()
        if kwargs is not None:
            for k, v in kwargs.iteritems():
                self.update(k, v)

    def query(self, variable, meta_info=None):
        if meta_info is not None:
            print "+++ Warning! Meta info field is ignored for local queries. +++"
        print "--- QUERY ---", variable, getattr(self, variable)

        return getattr(self, variable)

    def update(self, variable, value, meta_info=None):
        if meta_info is not None:
            print "+++ Warning! Meta info field is ignored for local updates. +++"
        setattr(self, variable, value)
        print "--- UPDATE ---", variable, getattr(self, variable)

    def __setattr__(self, name, value):
        with self.lock:
            self.__dict__["_mutex_"+name] = value

    def __getattr__(self, name):
        with self.lock:
            try:
                return self.__dict__["_mutex_"+name]
            except KeyError as e:
                return None

