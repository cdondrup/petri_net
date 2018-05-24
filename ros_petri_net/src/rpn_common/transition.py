from pn_common import PNBaseObject


class Arc(object):
    def __init__(self, place, weight=1):
        self.place = place
        self.weight = weight

    def __str__(self):
        return "{place}({weight})".format(place=self.place.name, weight=self.weight)

    def __repr__(self):
        return self.__str__()


class Transition(PNBaseObject):
    def __init__(self, name, incoming_arcs=None, outgoing_arcs=None, condition=True, action=None):
        super(Transition, self).__init__(name)
        self.condition = condition
        self.action = action
        self.incoming_arcs = incoming_arcs
        self.outgoing_arcs = outgoing_arcs

    def evaluate_condition(self):
        # Do something with the condition
        self.loginfo("Evaluating condition: '{}'".format(self.condition))
        try:
            return self.condition()
        except TypeError:
            self.loginfo("Condition is not callable, assuming boolean statement")
            return self.condition

    def execute_action(self):
        # Do someething to start condition
        if self.action:
            self.loginfo("Executing action: '{}'".format(self.action))
            self.action.start()
            self.loginfo("Action '{}' started.".format(self.action))
        else:
            self.loginfo("No action")
        return True

    def __str__(self):
        return "((" + self.name + ((", action: " + str(self.action)) if self.action else "") + \
                ", incoming: [" + ', '.join([str(x) for x in self.incoming_arcs]) + "], outgoing: [" + \
                ', '.join([str(x) for x in self.outgoing_arcs]) + "]" + ", condition: " + str(self.condition)  + "))"

    def __repr__(self):
        return self.__str__()

