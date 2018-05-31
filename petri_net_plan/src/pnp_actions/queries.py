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


class AbstractOperation(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self, internal_kb, external_kb):
        return

    def __call__(self, internal_kb, external_kb):
        return self.run(internal_kb, external_kb)


class Comparison(AbstractOperation):
    def __init__(self, operator, queries):
        self.queries = queries if isinstance(queries, list) else [queries]
        self.operator = operator

    def run(self, internal_kb, external_kb):
        values = []
        for q in self.queries:
            if isinstance(q, (LocalQuery, RemoteQuery)):
                if isinstance(q, LocalQuery):
                    kb = internal_kb
                elif isinstance(q, RemoteQuery):
                    kb = external_kb
                values.append(q(kb))
            elif isinstance(q, Query):
                r = q(internal_kb)
                values.append(r if r is not None else q(external_kb))
            else:
                values.append(q)
        return getattr(operator, self.operator)(*values)

    def __str__(self):
        return "{operator}({queries})".format(operator=self.operator, queries=', '.join(map(str,
                                                                                            self.queries)))


class BooleanAssertion(object):
    def __init__(self, operation, truth_value):
        self.operation = operation
        self.truth_value = truth_value

    def invert(self):
        self.truth_value = not self.truth_value

    def __call__(self, internal_kb, external_kb):
        return self.operation(internal_kb, external_kb) == self.truth_value

    def __str__(self):
        return "{} == {}".format(str(self.operation), str(self.truth_value))

