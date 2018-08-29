import operator
from abc import ABCMeta, abstractmethod


class AbstractAtomicUpdate(object):
    __metaclass__ = ABCMeta

    def __init__(self, attr, value, meta_info=None):
        self.attr = attr
        self.value = value
        self.meta_info = meta_info

    @abstractmethod
    def _run(self, kb, external_kb):
        return

    def _call_op(self, op, kb, external_kb):
        try:
            return op(kb, external_kb)
        except TypeError:
            return op

    def __call__(self, kb, external_kb):
        self._run(kb, external_kb)

    def __str__(self):
        return "{}({} <- {}, meta_info={})".format(self.__class__.__name__, str(self.attr), str(self.value), str(self.meta_info))

    def __repr__(self):
        return self.__str__()


class LocalUpdate(AbstractAtomicUpdate):
    def _run(self, kb, external_kb):
        kb.update(self.attr, self._call_op(self.value, kb, external_kb), meta_info=self.meta_info)


class RemoteUpdate(AbstractAtomicUpdate):
    def _run(self, kb, external_kb):
        external_kb.update(self.attr, self._call_op(self.value, kb, external_kb), meta_info=self.meta_info)


class Update(AbstractAtomicUpdate):
    def _run(self, kb, external_kb):
        kb.update(self.attr, self._call_op(self.value, kb, external_kb))
        external_kb.update(self.attr, self._call_op(self.value, kb, external_kb), meta_info=self.meta_info)

