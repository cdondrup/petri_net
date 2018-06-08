#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pnp_execution.executor import Executor
import yaml


class PetriNetNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        with open(rospy.get_param("~plan"), 'r') as f:
            self.plan = [x for x in yaml.load_all(f)]
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
        import pnp_kb.queries as queries
        from pnp_gen.operations import BooleanAssertion
        import pnp_gen.operations as operations
        import pnp_gen.logical_operations as logical_operations
        from pprint import pprint
        import numpy as np
        import importlib
        import inspect

        def __create_op(mutable, classes):
            if isinstance(mutable, list):
                for i, m in enumerate(mutable):
                    mutable[i] = __create_op(m, classes)
            elif isinstance(mutable, dict):
                for k in mutable.keys():
                    mutable[k] = __create_op(mutable[k], classes)
                for k, v in classes.iteritems():
                    if k in mutable:
                        return v(*(mutable.values()[0] if isinstance(mutable.values()[0], list) else [mutable.values()[0]]))
            return mutable

        domain, plan = plan

        query_members = dict(inspect.getmembers(queries, inspect.isclass))
        operations_members = dict(inspect.getmembers(operations, inspect.isclass))
        logical_operations_members = dict(inspect.getmembers(logical_operations, inspect.isclass))
        for k, v in logical_operations_members.items():
            key = k.lower().replace('logical', '')
            logical_operations_members[key] = v
        print logical_operations_members

        action_definitions = domain["actions"]
        action_definitions = __create_op(__create_op(__create_op(action_definitions, query_members), operations_members), logical_operations_members)
        for k, v in action_definitions.iteritems():
            action_definitions[k]["type"] = getattr(importlib.import_module(v["type"]["module"]), v["type"]["class"])
        print " --- Action Definitons: --- "
        pprint(action_definitions)

        gen = Generator()
        cp, net = gen.create_net("test_net1", ROSKB, initial_knowledge=plan["initial_knowledge"])

        plan = __create_op(__create_op(__create_op(plan["plan"], query_members), operations_members), logical_operations_members)
        print " --- Plan: --- "
        pprint(plan, width=120)

        def __create_action(action, action_definitions):
            print "ACTION", action
            name, params = action.items()[0]  # Only ever has one item
            ad = action_definitions[name]
            print "DEFINITION", ad

            b = []; a = []

            try:
                b.append(Before(BooleanAssertion(ad["preconditions"], False), Recovery.SKIP_ACTION))
            except KeyError:
                pass

            try:
                b.append(Before(BooleanAssertion(ad["effects"], True), Recovery.SKIP_ACTION))
                a.append(After(BooleanAssertion(ad["effects"], False), Recovery.RESTART_ACTION))
            except KeyError:
                pass

            return PNAction(
                ad["type"](name, params),
                recovery=Recovery(
                    before=b,
                    after=a
                )
            )

        def __create_concurrent_actions(actions, action_definitions):
            res  = []
            for action in actions.values()[0]:
                if "concurrent_actions" in action:
                    res.append(__create_concurrent_actions(action, action_definitions))
                else:
                    res.append(__create_action(action, action_definitions))
            return res

        def __parse_plan(cp, net, plan, action_definitions):
            for action in plan:
                if "while" in action:
                    print action
                    loop = action["while"]
                    print loop
                    acts = []
                    for a in loop["actions"]:
                        acts.append(__create_action(a, action_definitions))
                    cond = BooleanAssertion(loop["condition"], True)
                    cp, net = gen.add_loop(net, cp, acts, cond)
                elif "concurrent_actions" in action:
                    a = __create_concurrent_actions(action, action_definitions)
                    cp, net = gen.add_concurrent_actions(net, cp, a)
                else:
                    a = __create_action(action, action_definitions)
                    cp, net = gen.add_action(net, cp, a)

            return cp, net

        cp, net = __parse_plan(cp, net, plan, action_definitions)
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

