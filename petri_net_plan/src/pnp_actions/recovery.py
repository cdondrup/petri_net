from atomic_action import AtomicAction
from queries import BooleanAssertion, Comparison
import operator


class During(dict):
    OUTCOMES = AtomicAction.OUTCOMES
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES

    def __init__(self, preempted=None, failed=None):
        self[self.SUCCEEDED] = [None]  # Always move to the finish place
        self[self.PREEMPTED] = preempted if isinstance(preempted, list) else [preempted]
        self[self.FAILED] = failed if isinstance(failed, list) else [failed]


class Before(object):
    def __init__(self, assertion=None, recovery=None):
        self.assertion = assertion
        self.recovery = recovery if isinstance(recovery, list) else [recovery]


class After(object):
    def __init__(self, assertion=None, recovery=None):
        self.assertion = assertion
        self.recovery = recovery if isinstance(recovery, list) else [recovery]


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

