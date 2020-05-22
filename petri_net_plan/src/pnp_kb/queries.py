import operator
from abc import ABCMeta, abstractmethod


class AbstractAtomicQuery(object):
    __metaclass__ = ABCMeta

    def __init__(self, attr, meta_info=None):
        self.attr = attr
        self.meta_info = meta_info

    @abstractmethod
    def _run(self, kb, external_kb):
        return

    def _resolve_meta_info(self, kb, external_kb):
        try:
            for k in self.meta_info.keys():
                self.meta_info[k] = self._call_op(self.meta_info[k], kb, external_kb)
        except AttributeError:
            pass

    def _call_op(self, op, kb, external_kb):
        try:
            return op(kb, external_kb)
        except TypeError:
            return op

    def __call__(self, kb, external_kb):
        self._resolve_meta_info(kb, external_kb)
        return self._run(kb, external_kb)

    def __str__(self):
        return "{}({}, meta_info={})".format(self.__class__.__name__, str(self.attr), str(self.meta_info))

    def __repr__(self):
        return self.__str__()


class LocalQuery(AbstractAtomicQuery):
    def _run(self, kb, external_kb):
        return kb.query(self._call_op(self.attr, kb, external_kb), self.meta_info)


class RemoteQuery(AbstractAtomicQuery):
    def _run(self, kb, external_kb):
        return external_kb.query(self._call_op(self.attr, kb, external_kb), self.meta_info)


class Query(AbstractAtomicQuery):
    def _run(self, kb, external_kb):
        attr = self._call_op(self.attr, kb, external_kb)
        r = kb.query(attr, self.meta_info)
        return r if r is not None else external_kb.query(attr, self.meta_info)

