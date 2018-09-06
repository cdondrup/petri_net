#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import ActionServer
from ros_petri_net_msgs.msg import RPNAction, RPNResult
from pnp_execution.executor import Executor
import yaml
import json
from copy import deepcopy
from pnp_gen.generator import Generator
from pnp_actions.pn_action import PNAction
from pnp_actions.recovery import Recovery, Before, During, After
import pnp_kb.queries as queries
import pnp_kb.updates as updates
from pnp_gen.operations import BooleanAssertion
import pnp_gen.operations as operations
import pnp_gen.logical_operations as logical_operations
from pprint import pprint
import numpy as np
import importlib
import inspect
from threading import Thread


class PetriNetNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        self._as = ActionServer("/RPN", RPNAction, goal_cb=self.goal_cb, auto_start=False)
        # with open(rospy.get_param("~plan"), 'r') as f:
            # self.plan = [x for x in yaml.load_all(f)]
        # print self.plan
        self.exe = Executor()
        # exe.execute_net(exe.add_net(*self.create_net_from_plan(self.plan)))
        # exe.execute_net(exe.add_net(*self.test("test_net_1", {"value": 3})))
        # exe.execute_net(exe.add_net(*Generator().test("test_net_2", {"value": 4})))
        self._as.start()
        rospy.loginfo("Started '{}'.".format(name))

    def goal_cb(self, gh):
        gh.set_accepted()
        goal = gh.get_goal()
        net, marking = self.create_net_from_plan(goal.net_id, json.loads(goal.domain), json.loads(goal.plan))
        Thread(target=self.execute_net, args=(gh, net, marking)).start()

    def execute_net(self, gh, net, marking):
        self.exe.execute_net_and_wait(self.exe.add_net(net, marking))
        gh.set_succeeded()

    def __create_op(self, mutable, classes):
        if isinstance(mutable, list):
            for i, m in enumerate(mutable):
                mutable[i] = self.__create_op(m, classes)
        elif isinstance(mutable, dict):
            for k in mutable.keys():
                mutable[k] = self.__create_op(mutable[k], classes)
            for k, v in classes.iteritems():
                if k in mutable:
                    return v(*(mutable.values()[0] if isinstance(mutable.values()[0], list) else [mutable.values()[0]]))
        return mutable

    def __create_action(self, action, action_definitions):
        name, params = action.items()[0]  # Only ever has one item
        ad = action_definitions[name]

        b = []; a = []

        try:
            b.append(Before(BooleanAssertion(ad["preconditions"], False), Recovery.FAIL))
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
                during=During(failed=Recovery.FAIL),
                before=b,
                after=a
            )
        )

    def __create_concurrent_actions(self, actions, action_definitions):
        res  = []
        for action in actions.values()[0]:
            if "concurrent_actions" in action:
                res.append(self.__create_concurrent_actions(action, action_definitions))
            else:
                res.append(self.__create_action(action, action_definitions))
        return res

    def __parse_plan(self, cp, net, gen, plan, action_definitions):
        for action in plan:
            if "while" in action:
                loop = action["while"]
                acts = []
                for a in loop["actions"]:
                    acts.append(self.__create_action(a, action_definitions))
                cond = BooleanAssertion(loop["condition"], True)
                cp, net = gen.add_while_loop(net, cp, acts, cond)
            elif "concurrent_actions" in action:
                a = self.__create_concurrent_actions(action, action_definitions)
                cp, net = gen.add_concurrent_actions(net, cp, a)
            else:
                a = self.__create_action(action, action_definitions)
                cp, net = gen.add_action(net, cp, a)

        return cp, net

    def create_net_from_plan(self, net_id, domain, plan):
        external_kb = domain["external_knowledge_base"]
        external_kb = getattr(importlib.import_module(external_kb["module"]),external_kb["class"])

        members = dict(inspect.getmembers(queries, inspect.isclass))
        members.update(dict(inspect.getmembers(updates, inspect.isclass)))
        members.update(dict(inspect.getmembers(operations, inspect.isclass)))
        members.update(dict(inspect.getmembers(logical_operations, inspect.isclass)))
        for k, v in members.items():
            if "Logical" in k:
                key = k.lower().replace('logical', '')
                members[key] = v

        action_definitions = domain["actions"]
        action_definitions = self.__create_op(action_definitions, members)
        for k, v in action_definitions.iteritems():
            action_definitions[k]["type"] = getattr(importlib.import_module(v["type"]["module"]), v["type"]["class"])
        print " --- Action Definitons: --- "
        pprint(action_definitions)

        gen = Generator()
        cp, net = gen.create_net(net_id, external_kb, initial_knowledge=plan["initial_knowledge"])

        plan = self.__create_op(plan["plan"], members)
        print " --- Plan: --- "
        pprint(plan, width=120)

        cp, net = self.__parse_plan(cp, net, gen, plan, action_definitions)
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
        from pnp_kb.queries import LocalQuery, RemoteQuery, Query
        from pnp_gen.operations import BooleanAssertion, Comparison
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
    rospy.spin()

