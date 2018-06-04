from atomic_action import AtomicAction


class KBAction(AtomicAction):
    def start(self, kb ,external_kb):
        print "#############################", self.params
        kb.update(self.params["result"], self.params["operation"](kb, external_kb))
        print kb.query(self.params["result"])

    def monitor(self, kb, external_kb):
        # Instantaneous action
        return

    @property
    def succeeded(self):
        # Always succeed
        return True

    @property
    def preempted(self):
        return False

    @property
    def failed(self):
        return False
