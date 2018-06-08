import operator
from abc import ABCMeta, abstractmethod


class AbstractAtomicQuery(object):
    __metaclass__ = ABCMeta

    def __init__(self, attr):
        self.attr = attr

    def __call__(self, kb):
        return kb.query(self.attr)

    def __str__(self):
        return "{}({})".format(self.__class__.__name__, str(self.attr))

    def __repr__(self):
        return self.__str__()


class LocalQuery(AbstractAtomicQuery):
    pass


class RemoteQuery(AbstractAtomicQuery):
    pass


class Query(AbstractAtomicQuery):
    pass

