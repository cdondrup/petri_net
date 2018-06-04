#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pnp_execution.executor import Executor
import yaml


class PetriNetNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        with open(rospy.get_param("~plan"), 'r') as f:
            self.plan = yaml.load(f)
        print self.plan
        exe = Executor()
        exe.execute_net(exe.add_net(*self.create_net_from_plan(self.plan)))
        # exe.execute_net(exe.add_net(*self.test("test_net_1", {"value": 3})))
        # exe.execute_net(exe.add_net(*Generator().test("test_net_2", {"value": 4})))

    def create_net_from_plan(self, plan):
        from rpn_ros_interface.ros_external_knowledge_base import ROSExternalKnowledgeBase as ROSKB
        from pnp_gen.generator import Generator
        from pnp_actions.pn_action import PNAction
        from pnp_actions.recovery import Recovery, Before, During, After
        from pnp_actions.queries import LocalQuery, RemoteQuery, Query
        from pnp_actions.operations import BooleanAssertion
        import pnp_actions.operations as operations
        from pprint import pprint
        import numpy as np
        import importlib

        action_types = {k: getattr(importlib.import_module(v[:v.rfind('.')]), v[v.rfind('.')+1:]) for k, v in plan["action_types"].items()}
        action_definitions = plan["actions"]
        instances = plan["instances"]

        gen = Generator()
        cp, net = gen.create_net("test_net1", ROSKB, initial_knowledge=plan["initial_knowledge"])

        def __create_queries(operation, instances):
            operation = operation if isinstance(operation, list) else [operation]
            res = []
            for o in operation:
                if isinstance(o, list):
                    res.append(__create_queries(o, instances))
                else:
                    if o in instances:
                        res.append(Query(o))
                    else:
                        res.append(o)
            return res

        def __create_condition(type, condition, instances, truth_value, recovery):
                op = __create_queries(condition.values()[0], instances)
                print "OP", op
                return type(
                    BooleanAssertion(
                        getattr(operations, condition.keys()[0])(*op),
                        truth_value
                    ),
                    recovery
                )

        def __create_action(action, action_types, action_definitions, instances):
            print "ACTION", action
            action_definition = action_definitions[action.values()[0]["arguments"]["name"]]
            print "DEFINITION", action_definition
            b = []
            try:
                for pp in action_definition["positive_preconditions"]:
                    b.append(__create_condition(Before, pp, instances, False, Recovery.SKIP_ACTION))
            except KeyError:
                pass
            try:
                for np in action_definition["negative_preconditions"]:
                    b.append(__create_condition(Before, np, instances, True, Recovery.SKIP_ACTION))
            except KeyError:
                pass

            a = []
            try:
                for pe in action_definition["positive_effects"]:
                    a.append(__create_condition(After, pe, instances, False, Recovery.RESTART_ACTION))
            except KeyError:
                pass
            try:
                for ne in action_definition["negative_effects"]:
                    a.append(__create_condition(After, ne, instances, True, Recovery.RESTART_ACTION))
            except KeyError:
                pass

            return PNAction(
                action_types[action.keys()[0]](**action.values()[0]["arguments"]),
                recovery=Recovery(
                    before=b,
                    after=a
                )
            )

        def __create_concurrent_actions(actions, action_types, action_definitions, instances):
            res  = []
            for action in actions.values()[0]:
                if "concurrent_actions" in action:
                    res.append(__create_concurrent_actions(action, action_types, action_definitions, instances))
                else:
                    res.append(__create_action(action, action_types, action_definitions, instances))
            return res

        for action in plan["plan"]:
            if "concurrent_actions" in action:
                a = __create_concurrent_actions(action, action_types, action_definitions, instances)
                cp, net = gen.add_concurrent_actions(net, cp, a)
            else:
                a = __create_action(action, action_types, action_definitions, instances)
                cp, net = gen.add_action(net, cp, a)

        cp, net = gen.add_goal(net, cp)
        pprint(net)
        marking = np.zeros(net.num_places, dtype=int)
        marking[0] = 1
        return net, marking


    def test(self, name, initial_knowledge):
        from rpn_ros_interface.action import ROSAtomicAction as ROSAction
        from rpn_ros_interface.ros_external_knowledge_base import ROSExternalKnowledgeBase as ROSKB
        from pnp_gen.generator import Generator
        from pnp_actions.pn_action import PNAction
        from pnp_actions.recovery import Recovery, Before, During, After
        from pnp_actions.queries import BooleanAssertion, Comparison, LocalQuery, RemoteQuery, Query
        from pprint import pprint
        import numpy as np

        gen = Generator()
        cp, net = gen.create_net(name, ROSKB, initial_knowledge=initial_knowledge)
        a1 = PNAction(
            atomic_action=ROSAction("dummy_server"),
            recovery=Recovery(
                before=[
                    Before(
                        BooleanAssertion(
                            Comparison("eq", [Query("time"), 3]),
                            True
                        ),
                        Recovery.SKIP_ACTION),
                    Before(
                        BooleanAssertion(
                            Comparison("eq", [Query("time"), 2]),
                            True
                        ),
                        [
                            PNAction(ROSAction("wait", {"time": 1})),
                            PNAction(ROSAction("dummy_server")),
                            Recovery.SKIP_ACTION
                        ]
                    )
                ],
                during=During(
                    preempted=[
                        PNAction(
                            ROSAction("dummy_server"),
                            recovery=Recovery(
                                during=During(
                                    preempted=[
                                        PNAction(ROSAction("wait", {"time": 2})),
                                        Recovery.SKIP_ACTION
                                    ]
                                )
                            )
                        ),
                        Recovery.SKIP_ACTION
                    ],
                    failed=Recovery.FAIL
                ),
                after=[
                    After(
                        BooleanAssertion(
                            Comparison("eq", [LocalQuery("time"), LocalQuery("value")]),
                            False
                        ),
                        Recovery.RESTART_ACTION
                    ),
                    After(
                        BooleanAssertion(
                            Comparison("eq", [Query("time"), Query("UNKNOWN ARGUMENT")]),
                            False
                        ),
                        [
                            PNAction(ROSAction("wait", {"time": 1})),
                            Recovery.SKIP_ACTION
                        ]
                    )
                ]
            )
        )
        cp, net = gen.add_action(net, cp, a1)

        a21 = PNAction(ROSAction("wait"))
        a22 = PNAction(ROSAction("wait"))
        a31 = PNAction(ROSAction("wait", {"time": 5}))
        a32 = PNAction(ROSAction("wait", {"time": 6}))
        cp, net = gen.add_concurrent_actions(net, cp, [[a31, a32], a21, a22])

        cp, net = gen.add_goal(net, cp)
        pprint(net)
        marking = np.zeros(net.num_places, dtype=int)
        marking[0] = 1
        return net, marking


if __name__ == "__main__":
    rospy.init_node("ros_petri_net_node")
    p = PetriNetNode(rospy.get_name())
    # rospy.spin()

