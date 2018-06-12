from abc import ABCMeta, abstractmethod


class AbstractKnowledgeBase(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def query(self, variable, meta_info=None):
        return

    @abstractmethod
    def update(self, variable, value, meta_info=None):
        pass

