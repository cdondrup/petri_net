from atomic_action import AtomicAction
import operator


class During(dict):
    OUTCOMES = AtomicAction.OUTCOMES
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES

    def __init__(self, preempted=None, failed=None):
        self[self.SUCCEEDED] = None  # Always move to the finish place
        self[self.PREEMPTED] = preempted
        self[self.FAILED] = failed


class Check(object):
    def __init__(self, value1, operator, value2):
        self.value1 = value1
        self.operator = operator
        self.value2 = value2

    def __call__(self, kb):
        try:
            self.value1 = kb.query(self.value1)
        except (TypeError, KeyError):
            pass
        try:
            self.value2 = kb.query(self.value2)
        except (TypeError, KeyError):
            pass
        return getattr(operator, self.operator)(self.value1, self.value2)

    def __str__(self):
        return "{operator}({value1}, {value2})".format(**self.__dict__)


class BooleanTest(object):
    def __init__(self, test, truth_value):
        self.test = test
        self.truth_value = truth_value

    def invert(self):
        self.truth_value = not self.truth_value

    def __str__(self):
        return "{} == {}".format(str(self.test), str(self.truth_value))


class Before(object):
    def __init__(self, boolean_test=None, recovery=None):
        self.boolean_test = boolean_test
        self.recovery = recovery


class After(object):
    def __init__(self, boolean_test=None, recovery=None):
        self.boolean_test = boolean_test
        self.recovery = recovery


class Recovery(object):
    RESTART_ACTION = "restart_action"
    RESTART_PLAN = "restart_plan"
    SKIP_ACTION = "skip_action"
    FAIL = "fail"

    def __init__(self, before=None, during=None, after=None):
        if before is not None:
            before = before if isinstance(before, list) else [before]
            for b in before:
                if not isinstance(b, Before):
                    raise TypeError("Recovery behaviours to be executed before an action have to be of type Before")
            self.before = before
        else:
            self.before = []
        if during is not None:
            if isinstance(during, During):
                self.during = during
            else:
                raise TypeError("Recovery behaviours to be executed before an action have to be of type During")
        else:
            self.during = During()
        if after is not None:
            after = after if isinstance(after, list) else [after]
            for a in after:
                if not isinstance(a, After):
                    raise TypeError("Recovery behaviours to be executed before an action have to be of type After")
            self.after = after
        else:
            self.after = []

