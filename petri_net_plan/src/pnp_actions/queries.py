import operator
from abc import ABCMeta, abstractmethod


class AbstractAtomicQuery(object):
    __metaclass__ = ABCMeta

    def __init__(self, attr):
        self.attr = attr

    def __call__(self, kb):
        return kb.query(self.attr)

    def __str__(self):
        return str(self.attr)


class LocalQuery(AbstractAtomicQuery):
    pass


class RemoteQuery(AbstractAtomicQuery):
    pass


class Query(AbstractAtomicQuery):
    pass

