import operator
from abc import ABCMeta, abstractmethod
from pnp_kb.queries import Query, LocalQuery, RemoteQuery


class AbstractOperation(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        return

    def execute_query(self, query):
        if isinstance(query, (Query, LocalQuery, RemoteQuery)):
            return query(self.internal_kb, self.external_kb)
        else:
            return query

    def __call__(self, internal_kb, external_kb):
        self.internal_kb = internal_kb
        self.external_kb = external_kb
        return self.run()

    def __repr__(self):
        return self.__str__()


class Exists(AbstractOperation):
    def __init__(self, query):
        self.query = query

    def run(self):
        return self.execute_query(self.query) is not None

    def __str__(self):
        return "{operator}({query})".format(operator=self.__class__.__name__, query=str(self.query))


class Operation(AbstractOperation):
    def __init__(self, operator, queries):
        self.queries = queries if isinstance(queries, list) else [queries]
        self.operator = operator

    def run(self):
        values = []
        for q in self.queries:
            values.append(self.execute_query(q))
        r = getattr(operator, self.operator)(*values)
        print "--- Operation ---: {} = {}".format(str(self), str(r))
        print values
        return r

    def __str__(self):
        return "{operator}({queries})".format(operator=self.operator, queries=', '.join(map(str, self.queries)))


class Comparison(Operation):
    """
    Operation is able to do comparisons as well but we want to make it a little more readable.
    """

    def __init__(self, operator, queries):
        super(Comparison, self).__init__(operator, queries)
        if len(self.queries) != 2:
            raise AttributeError("A comparison needs exactly two arguments.")


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

