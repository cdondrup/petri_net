from recovery import Recovery


class DuringExecutionRecovery(Recovery):
    OUTCOMES = ("succeeded", "preempted", "failed")
    SUCCEEDED, PREEMPTED, FAILED = OUTCOMES

    POSSIBLE_ACTIONS = ("fail", "restart")
    FAIL, RESTART = POSSIBLE_ACTIONS

    def __init__(self, succeeded=None, preempted=None, failed=None):
        super(DuringExecutionRecovery, self).__init__()
        self.recovery = {o: None for o in self.OUTCOMES}
        self.recovery[self.SUCCEEDED] = succeeded
        self.recovery[self.PREEMPTED] = preempted
        self.recovery[self.FAILED] = failed

