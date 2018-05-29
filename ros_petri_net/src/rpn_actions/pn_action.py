# from abc import ABCMeta, abstractmethod
from uuid import uuid4
from recovery import Recovery
from rpn_common.transition import Transition, Arc
from rpn_common.place import Place


class PNAction(object):
    def __init__(self, atomic_action, recovery=None):
        self.id = str(uuid4())
        self.atomic_action = atomic_action
        self.places = []
        self.transitions = []
        self.recovery = recovery if recovery is not None else Recovery()

        self.exec_place = Place(name=atomic_action.name+".exec", atomic_action=atomic_action)
        self.end_place = Place(name=atomic_action.name+".finished")
        self.places = [self.exec_place, self.end_place]

        self.starting_transition = Transition(
            name=atomic_action.name+".start",
            atomic_action=atomic_action,
            outgoing_arcs=[Arc(place=self.exec_place)]
        )
        self.transitions = [self.starting_transition]

    def add_starting_place(self, place):
        self.starting_transition = self.starting_transition if isinstance(self.starting_transition,
                                                                          list) else [self.starting_transition]
        for t in self.starting_transition:
            t.incoming_arcs = [Arc(place=place)]

    def apply_recovery_behaviours(self, net, cp):
        # Before
        for i, b in enumerate(self.recovery.before.checks):
            p = Place("P.before{}".format(str(i)))
            self.add_starting_place(p)
            self.starting_transition = Transition(
                name="T.before"+str(i),
                condition=lambda: b(True),
                outgoing_arcs=[Arc(place=p)]
            )
            self.places.append(p)
            self.transitions.append(self.starting_transition)
            cp = p

        # During
        for o, r in self.recovery.during.items():
            if r == Recovery.RESTART_ACTION:
                pr = cp
            elif r == Recovery.RESTART_PLAN:
                pr = net.init_place
            elif r == Recovery.FAIL:
                pr = Place("Fail")
                self.places.append(pr)
            elif r is None:
                pr = self.end_place
            self.transitions.append(Transition(
                name=self.atomic_action.name+"."+o,
                condition=(lambda x: lambda: getattr(self.atomic_action, x))(o),
                incoming_arcs=[Arc(place=self.exec_place)],
                outgoing_arcs=[Arc(place=pr)]
            ))

    def add_places(self, places):
        self.places.extend(places if isinstance(places, list) else [places])

    def add_transitions(self, transitions):
        self.transitions.extend(transitions if isinstance(transitions, list) else [transitions])

