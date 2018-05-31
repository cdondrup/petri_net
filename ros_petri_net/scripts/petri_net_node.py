#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pnp_execution.executor import Executor


class PetriNetNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        exe = Executor()
        exe.execute_net(exe.add_net(*self.test("test_net_1", {"value": 3, "time": 1})))
        # exe.execute_net(exe.add_net(*Generator().test("test_net_2", {"value": 4})))

    def test(self, name, initial_knowledge):
        from rpn_ros_interface.action import ROSAtomicAction as ROSAction
        from pnp_gen.generator import Generator
        from pnp_actions.pn_action import PNAction
        from pnp_actions.recovery import Recovery, Before, During, After, BooleanTest, Check
        from pprint import pprint
        import numpy as np

        gen = Generator()
        cp, net = gen.create_net(name, initial_knowledge=initial_knowledge)
        a1 = PNAction(
            atomic_action=ROSAction("dummy_server"),
            recovery=Recovery(
                before=Before(BooleanTest(Check("time", "eq", 3), True), Recovery.SKIP_ACTION),
                during=During(
                    preempted=Recovery.RESTART_PLAN,
                    failed=Recovery.FAIL
                ),
                after=After(BooleanTest(Check("time", "eq", "value"), False), Recovery.RESTART_ACTION)
            )
        )
        a21 = PNAction(ROSAction("wait"))
        a22 = PNAction(ROSAction("wait"))
        a31 = PNAction(ROSAction("wait", {"time": 5}))
        a32 = PNAction(ROSAction("wait", {"time": 6}))
        cp, net = gen.add_action(net, cp, a1)
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

