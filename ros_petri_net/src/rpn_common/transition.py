from pn_common import PNBaseObject


class Transition(PNBaseObject):
    def __init__(self, name, condition=None, action=None):
        super(Transition, self).__init__(name)
        self.condition = condition
        self.action = action

    def evaluate_condition(self):
        # Do something with the condition
        self.loginfo("Evaluating condition: '{}'".format(self.condition))
        return self.condition

    def execute_action(self):
        # Do someething to start condition
        if self.action:
            self.loginfo("Executiing action: '{}'".format(self.action))
        else:
            self.loginfo("No action")
        return True

