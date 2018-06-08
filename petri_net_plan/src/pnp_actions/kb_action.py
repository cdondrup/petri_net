from atomic_action import AtomicAction


class KBAction(AtomicAction):
    def run(self, kb ,external_kb):
        with self.__mutex__:
            kb.update(self.params["result"], self.params["operation"](kb, external_kb))
            print kb.query(self.params["result"])

    @property
    def succeeded(self):
        # Always succeed
        with self.__mutex__:
            return True

    @property
    def preempted(self):
        with self.__mutex__:
            return False

    @property
    def failed(self):
        with self.__mutex__:
            return False
