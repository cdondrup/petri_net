from pnp_common.transition import Transition, Arc
from pnp_common.place import Place
from pnp_common.petri_net import PetriNet
from pnp_kb.knowledgebase import KnowledgeBase
from pprint import pprint
import numpy as np


class Generator(object):
    PLACES = "places"
    TRANSITIONS = "transitions"
    D_MINUS = "d-"
    D_PLUS = "d+"
    D = "d"

    def __init__(self):
        self.__kb = KnowledgeBase()
        self.__place_counter = 0
        self.__trans_counter = 0

    def create_net(self, name, external_kb, initial_knowledge=None):
        p = Place("Init")
        net = PetriNet(name, external_kb=external_kb, initial_knowledge=initial_knowledge)
        net.add_init_place(p)

        return p, net

    def create_place(self, name=None, atomic_action=None):
        if name is None:
            name = "P{}".format(str(self.__place_counter))
            self.__place_counter += 1

        return Place(name=name, atomic_action=atomic_action)

    def create_transition(self, name=None, atomic_action=None, condition=True, incoming_arcs=None, outgoing_arcs=None):
        if name is None:
            name = "T{}".format(str(self.__trans_counter))
            self.__trans_counter += 1

        return Transition(name=name, atomic_action=atomic_action, incoming_arcs=incoming_arcs,
                          outgoing_arcs=outgoing_arcs, condition=condition)

    def add_action(self, net, current_place, action):
        action.set_up(net)
        net.add_transition(self.create_transition(
            incoming_arcs=[Arc(place=current_place)],
            outgoing_arcs=[Arc(place=action.start_place)]
        ))

        for p in action.places:
            net.add_place(p)
        for t in action.transitions:
            net.add_transition(t)
        return action.end_place, net

    def add_concurrent_actions(self, net, current_place, actions):
        fork_ps, fork_t = self.create_fork(current_place, len(actions))
        net.add_transition(fork_t)
        join_p, join_t = self.create_join()

        for p, a in zip(fork_ps, actions):
            net.add_place(p)
            if isinstance(a, list):
                ap, net = self.add_concurrent_actions(net, p, a)
            else:
                ap, net = self.add_action(net, p, a)
            join_t.incoming_arcs.append(Arc(place=ap))

        net.add_transition(join_t)
        net.add_place(join_p)
        return join_p, net

    def create_fork(self, current_place, num):
        ps = [self.create_place() for _ in range(num)]
        t = self.create_transition(
            incoming_arcs=[Arc(place=current_place)],
            outgoing_arcs=[Arc(place=p) for p in ps]
        )

        return ps, t

    def create_join(self):
        p = self.create_place()
        t = self.create_transition(
            incoming_arcs=[],
            outgoing_arcs=[Arc(place=p)]
        )

        return p, t

    def create_fail_place(self):
        return self.create_place("Fail")

    def add_goal(self, net, current_place):
        p = self.create_place(name="Goal")
        net.add_place(p)
        net.add_transition(
            self.create_transition(
                incoming_arcs=[Arc(place=current_place)],
                outgoing_arcs=[Arc(place=p)]
            )
        )
        return p, net

