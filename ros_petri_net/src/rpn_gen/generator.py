from rpn_common.transition import Transition, Arc
from rpn_common.place import Place
from rpn_common.petri_net import PetriNet
from rpn_actions.atomic_action import AtomicAction
from rpn_actions.pn_action import PNAction
from rpn_kb.knowledgebase import KnowledgeBase
from pprint import pprint
import numpy as np


class Generator(object):
    PLACES = "places"
    TRANSITIONS = "transitions"
    D_MINUS = "d-"
    D_PLUS = "d+"
    D = "d"

    RESTART = "restart"
    FAIL = "fail"

    def __init__(self):
        self.__kb = KnowledgeBase()
        self.__place_counter = 0
        self.__trans_counter = 0

    def test(self, name, initial_knowledge):
        from rpn_ros_interface.action import ROSAtomicAction as ROSAction
        cp, net = self.create_net(name, initial_knowledge=initial_knowledge)
        cp, net = self.add_action(
            net,
            cp,
            ROSAction(
                "dummy_server",
                recovery={
                    AtomicAction.PREEMPTED: Generator.RESTART,
                    AtomicAction.FAILED: Generator.FAIL
                }
            )
        )
        cp, net = self.add_concurrent_actions(
            net,
            cp,
            [
                [
                    ROSAction("wait", {"time": 5}),
                    ROSAction("wait", {"time": 6})
                ],
                ROSAction("wait"),
                ROSAction("wait")
            ]
        )
        cp, net = self.add_goal(net, cp)
        pprint(net)
        marking = np.zeros(net.num_places, dtype=int)
        marking[0] = 1
        return net, marking


    def create_net(self, name, initial_knowledge=None):
        p = Place("Init")
        net = PetriNet(name, initial_knowledge=initial_knowledge)
        net.add_place(p)

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

    def create_action(self, atomic_action):
        p = self.create_place(name=atomic_action.name+".exec", atomic_action=atomic_action)
        p2 = self.create_place(name=atomic_action.name+".finished")
        t = self.create_transition(
            name=atomic_action.name+".start",
            atomic_action=atomic_action,
            outgoing_arcs=[Arc(place=p)]
        )
        t2 = [t]
        for outcome, recovery in atomic_action.recovery.items():
            # if recovery == "fail":
                # pr = self.create_place(name="Fail")
                # net.add_place(pr)
            # elif recovery == "restart":
                # pr = current_place
            # else:
                # pr = p2
            t2.append(self.create_transition(
                name=atomic_action.name+"."+outcome,
                condition=(lambda x: lambda: getattr(atomic_action, x))(outcome),
                incoming_arcs=[Arc(place=p)],
                # outgoing_arcs=[Arc(place=pr)]
                outgoing_arcs=[Arc(place=p2)]
            ))
        return PNAction(
            atomic_action=atomic_action,
            starting_transition=t,
            end_place=p2,
            places=[p, p2],
            transitions=t2
        )

    def add_action(self, net, current_place, action):
        action = self.create_action(action)
        # p = self.create_place(name=action.name+".exec", atomic_action=action)
        action.starting_transition.incoming_arcs = [Arc(place=current_place)]
        for p in action.places:
            net.add_place(p)
        for t in action.transitions:
            net.add_transition(t)
            # self.create_transition(
                # name=action.name+".start",
                # atomic_action=action,
                # incoming_arcs=[Arc(place=current_place)],
                # outgoing_arcs=[Arc(place=p)]
            # )
        # )

        # p2 = self.create_place(name=action.name+".finished")
        # net.add_place(p2)
        # for outcome, recovery in action.recovery.items():
            # if recovery == "fail":
                # pr = self.create_place(name="Fail")
                # net.add_place(pr)
            # elif recovery == "restart":
                # pr = current_place
            # else:
                # pr = p2
            # net.add_transition(
                # self.create_transition(
                    # name=action.name+"."+outcome,
                    # condition=(lambda x: lambda: getattr(action, x))(outcome),
                    # incoming_arcs=[Arc(place=p)],
                    # outgoing_arcs=[Arc(place=pr)]
                # )
            # )

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

