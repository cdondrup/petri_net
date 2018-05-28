from pn_common import PNBaseObject
import inspect


class Arc(object):
    def __init__(self, place, weight=1):
        self.place = place
        self.weight = weight

    def __str__(self):
        return "{place}({weight})".format(place=self.place.name, weight=self.weight)

    def __repr__(self):
        return self.__str__()


class Transition(PNBaseObject):
    def __init__(self, name, incoming_arcs=None, outgoing_arcs=None, condition=True, atomic_action=None):
        super(Transition, self).__init__(name)
        self.condition = condition
        self.atomic_action = atomic_action
        self.incoming_arcs = incoming_arcs
        self.outgoing_arcs = outgoing_arcs

    def evaluate_condition(self):
        self.loginfo("Evaluating condition")
        try:
            return self.condition()
        except TypeError:
            self.loginfo("Condition is not callable, assuming boolean statement")
            return self.condition

    def execute_atomic_action(self):
        if self.atomic_action:
            self.loginfo("Executing atomic_action: '{}'".format(self.atomic_action))
            self.atomic_action.start(self.kb)
            self.loginfo("Action '{}' started.".format(self.atomic_action))
        else:
            self.loginfo("No atomic_action")
        return True

    def __str__(self):
        return "((" + self.name + ((", atomic_action: " + str(self.atomic_action)) if self.atomic_action else "") + \
                ", incoming: [" + ', '.join([str(x) for x in self.incoming_arcs]) + "], outgoing: [" + \
                ', '.join([str(x) for x in self.outgoing_arcs]) + "]" + ", condition: " + str(self.condition)  + "))"

    def __repr__(self):
        return self.__str__()
