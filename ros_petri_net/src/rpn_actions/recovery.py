from atomic_action import AtomicAction


class During(dict):
    OUTCOMES = AtomicAction.OUTCOMES
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES

    def __init__(self, preempted=None, failed=None):
        self[self.SUCCEEDED] = None  # Always move to the finish place
        self[self.PREEMPTED] = preempted
        self[self.FAILED] = failed


class Check(object):
    def __init__(self, attr, operator, value, recovery):
        self.attr = attr
        self.operator = operator
        self.value = value
        self.recovery=recovery

    def __call__(self, comp):
        return getattr(self.attr, self.operator)(self.value) == comp


class Before(object):
    def __init__(self, *args):
        self.checks = args


class After(object):
    def __init__(self, *args):
        self.checks = args


class Recovery(object):
    RESTART_ACTION = "restart_action"
    RESTART_PLAN = "restart_plan"
    SKIP_ACTION = "skip_action"
    FAIL = "fail"

    def __init__(self, before=None, during=None, after=None):
        if before is not None:
            if isinstance(before, Before):
                self.before = before
            else:
                raise TypeError("Recovery behaviours have to be of type Before")
        else:
            self.before = Before()
        if during is not None:
            if isinstance(during, During):
                self.during = during
            else:
                raise TypeError("Recovery behaviours have to be of type During")
        else:
            self.during = During()
        if after is not None:
            if isinstance(after, After):
                self.after = after
            else:
                raise TypeError("Recovery behaviours have to be of type After")
        else:
            self.after = After()

