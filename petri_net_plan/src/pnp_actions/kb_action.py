from atomic_action import AtomicAction


class KBAction(AtomicAction):
    def run(self, kb, external_kb):
        with self.__mutex__:
            self.params["operation"](kb, external_kb)

    def get_state(self):
        if self.__mutex__.acquire(False):
            try:
                return True  # Always succeed
            except:
                return False
            finally:
                self.__mutex__.release()
        return False

    @property
    def succeeded(self):
        return self.get_state()

    @property
    def preempted(self):
        return False

    @property
    def failed(self):
        return False
