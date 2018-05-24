from rpn_common.transition import Transition, Arc
from rpn_common.place import Place
from rpn_common.petri_net import PetriNet
from rpn_ros_interface.action import Action
from pprint import pprint
import numpy as np


class Generator(object):
    PLACES = "places"
    TRANSITIONS = "transitions"
    D_MINUS = "d-"
    D_PLUS = "d+"
    D = "d"

    def __init__(self):
        self.__place_counter = 0
        self.__trans_counter = 0

    def test(self, name):
        cp, net = self.create_net(name)
        # cp, net = self.add_action(net, cp, "action_1")
        # cp, net = self.add_action(net, cp, "action_2")
        cp, net = self.add_concurrent_actions(
            net,
            cp,
            [
                [
                    Action("wait", {"time": 5}),
                    Action("wait", {"time": 6})
                ],
                Action("wait", {"time": 3}),
                Action("wait", {"time": 4})
            ]
        )
        cp, net = self.add_goal(net, cp)
        pprint(net)
        marking = np.zeros(net.num_places, dtype=int)
        marking[0] = 1
        return net, marking


    def create_net(self, name):
        p = Place("Init")
        net = PetriNet(name)
        net.add_place(p)

        return p, net

    def create_place(self, name=None, action=None):
        if name is None:
            name = "P{}".format(str(self.__place_counter))
            self.__place_counter += 1

        return Place(name=name, action=action)

    def create_transition(self, name=None, action=None, condition=True, incoming_arcs=None, outgoing_arcs=None):
        if name is None:
            name = "T{}".format(str(self.__trans_counter))
            self.__trans_counter += 1

        return Transition(name=name, action=action, incoming_arcs=incoming_arcs,
                          outgoing_arcs=outgoing_arcs, condition=condition)

    def add_action(self, net, current_place, action):
        p = self.create_place(name=action.name+".exec", action=action)
        net.add_place(p)
        net.add_transition(
            self.create_transition(
                name=action.name+".start",
                action=action,
                incoming_arcs=[Arc(place=current_place)],
                outgoing_arcs=[Arc(place=p)]
            )
        )

        p2 = self.create_place(name=action.name+".succeeded")
        net.add_place(p2)
        net.add_transition(
            self.create_transition(
                name=action.name+".end",
                condition=lambda: action.succeeded,
                incoming_arcs=[Arc(place=p)],
                outgoing_arcs=[Arc(place=p2)]
            )
        )

        return p2, net

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

