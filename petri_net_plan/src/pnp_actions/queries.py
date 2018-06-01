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
    def run(self):
        return

    def execute_query(self, query):
        if isinstance(query, (LocalQuery, RemoteQuery)):
            if isinstance(query, LocalQuery):
                kb = self.internal_kb
            elif isinstance(query, RemoteQuery):
                kb = self.external_kb
            return query(kb)
        elif isinstance(query, Query):
            r = query(self.internal_kb)
            return r if r is not None else query(self.external_kb)
        else:
            return query


    def __call__(self, internal_kb, external_kb):
        self.internal_kb = internal_kb
        self.external_kb = external_kb
        return self.run()


class Exists(AbstractOperation):
    def __init__(self, query):
        self.query = query

    def run(self):
        return self.execute_query(self.query) is not None


class Comparison(AbstractOperation):
    def __init__(self, operator, queries):
        self.queries = queries if isinstance(queries, list) else [queries]
        self.operator = operator

    def run(self):
        values = []
        for q in self.queries:
            values.append(self.execute_query(q))
        return getattr(operator, self.operator)(*values)

    def __str__(self):
        return "{operator}({queries})".format(operator=self.operator, queries=', '.join(map(str, self.queries)))


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

