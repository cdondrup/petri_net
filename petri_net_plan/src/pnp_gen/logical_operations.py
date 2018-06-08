from abc import ABCMeta, abstractmethod


class AbstractLogicalOperation(object):
    __metaclass__ = ABCMeta

    def __init__(self, *args):
        self.operations = args

    @abstractmethod
    def run(self):
        return

    def __call__(self, internal_kb, external_kb):
        self.internal_kb = internal_kb
        self.external_kb = external_kb
        return self.run()

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return ' {} '.join(map(str, self.operations)).format(self.__class__.__name__.lower().replace('logical',''))


class LogicalAnd(AbstractLogicalOperation):
    def run(self):
        for o in self.operations:
            if not o(self.internal_kb, self.external_kb):
                return False
        return True


class LogicalOr(AbstractLogicalOperation):
    def run(self):
        for o in self.operations:
            if o(self.internal_kb, self.external_kb):
                return True
        return False


class LogicalNot(AbstractLogicalOperation):
    def __init__(self, *args):
        if len(args) != 1:
            raise AttributeError("The logical 'not' can only have one argument.")
        super(LogicalNot, self).__init__(*args)

    def run(self):
        return not self.operations[0](self.internal_kb, self.external_kb)

    def __str__(self):
        return "not({})".format(str(self.operations[0]))

