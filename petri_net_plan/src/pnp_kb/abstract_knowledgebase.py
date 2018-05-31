from abc import ABCMeta, abstractmethod


class AbstractKnowledgeBase(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def query(self, variable):
        return

    @abstractmethod
    def update(self, variable, value):
        pass

