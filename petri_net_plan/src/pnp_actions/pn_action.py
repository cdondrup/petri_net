# from abc import ABCMeta, abstractmethod
from uuid import uuid4
from recovery import Recovery
from pnp_common.transition import Transition, Arc
from pnp_common.place import Place
from copy import deepcopy


class PNAction(object):
    def __init__(self, atomic_action, recovery=None):
        self.id = str(uuid4())
        self.atomic_action = atomic_action
        self.places = []
        self.transitions = []
        self.recovery = recovery if recovery is not None else Recovery()

        self.start_place = Place(name=atomic_action.name+".P.start")
        self.exec_place = Place(name=atomic_action.name+".P.exec", atomic_action=atomic_action)
        self.end_place = Place(name=atomic_action.name+".P.finished")
        self.places = [self.start_place, self.exec_place, self.end_place]

        self.transitions = [Transition(
            name=atomic_action.name+".T.start",
            atomic_action=atomic_action,
            incoming_arcs=[Arc(place=self.start_place)],
            outgoing_arcs=[Arc(place=self.exec_place)]
        )]

    # def add_starting_place(self, place):
        # self.start_transition = self.start_transition if isinstance(self.start_transition,
                                                                          # list) else [self.start_transition]
        # for t in self.start_transition:
            # t.incoming_arcs = [Arc(place=place)]

    def apply_recovery_behaviours(self, net):
        # During
        for o, r in self.recovery.during.items():
            if r == Recovery.RESTART_ACTION:
                pr = self.start_place
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

        # Before
        for i, b in enumerate(self.recovery.before):
            p = Place(self.atomic_action.name+".P.before{}".format(str(i)))
            self.places.append(p)

            r = b.recovery
            if r == Recovery.SKIP_ACTION:
                pr = "INSERT END PLACE"
            elif r == Recovery.RESTART_PLAN:
                pr = net.init_place
            elif r == Recovery.FAIL:
                pr = Place("Fail")
                self.places.append(pr)
            else:
                raise AttributeError("Recovery '{}' is not supported as 'before' recovery".format(r))

            self.transitions.append(Transition(
                name=self.atomic_action.name+".T.before{}.{}".format(str(i), str(b.boolean_test.truth_value)),
                query=b.boolean_test,
                incoming_arcs=[Arc(place=p)],
                outgoing_arcs=[Arc(place=pr)]
            ))
            inverse_bt = deepcopy(b.boolean_test)
            inverse_bt.invert()
            self.transitions.append(Transition(
                name=self.atomic_action.name+".T.before{}.{}".format(str(i), str(inverse_bt.truth_value)),
                query=inverse_bt,
                incoming_arcs=[Arc(place=p)],
                outgoing_arcs=[Arc(place=self.start_place)]
            ))
            self.start_place = p

        # After
        for i, a in enumerate(self.recovery.after):
            p = Place(self.atomic_action.name+".P.after{}".format(str(i)))
            self.places.append(p)

            r = a.recovery
            if r == Recovery.RESTART_ACTION:
                pr = "INSERT START PLACE"
            elif r == Recovery.RESTART_PLAN:
                pr = net.init_place
            elif r == Recovery.FAIL:
                pr = Place("Fail")
                self.places.append(pr)
            else:
                raise AttributeError("Recovery '{}' is not supported as 'after' recovery".format(r))

            self.transitions.append(Transition(
                name=self.atomic_action.name+".T.after{}.{}".format(str(i), str(a.boolean_test.truth_value)),
                query=a.boolean_test,
                incoming_arcs=[Arc(place=self.end_place)],
                outgoing_arcs=[Arc(place=pr)]
            ))
            inverse_at = deepcopy(a.boolean_test)
            inverse_at.invert()
            self.transitions.append(Transition(
                name=self.atomic_action.name+".T.after{}.{}".format(str(i), str(inverse_at.truth_value)),
                query=inverse_at,
                incoming_arcs=[Arc(place=self.end_place)],
                outgoing_arcs=[Arc(place=p)]
            ))
            self.end_place = p

        for t in self.transitions:
            for a in t.incoming_arcs:
                if a.place == "INSERT END PLACE":
                    a.place = self.end_place
                if a.place == "INSERT START PLACE":
                    a.place = self.start_place
            for a in t.outgoing_arcs:
                if a.place == "INSERT END PLACE":
                    a.place = self.end_place
                if a.place == "INSERT START PLACE":
                    a.place = self.start_place

    def add_places(self, places):
        self.places.extend(places if isinstance(places, list) else [places])

    def add_transitions(self, transitions):
        self.transitions.extend(transitions if isinstance(transitions, list) else [transitions])

