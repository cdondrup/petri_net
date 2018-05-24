#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rpn_gen.generator import Generator
from rpn_execution.executor import Executor


class PetriNetNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        exe = Executor()
        exe.execute_net(exe.add_net(*Generator().test("test_net_1")))
        # exe.execute_net(exe.add_net(*Generator().test("test_net_3")))
        # exe.execute_net(exe.add_net(*Generator().test("test_net_2")))


if __name__ == "__main__":
    rospy.init_node("ros_petri_net_node")
    p = PetriNetNode(rospy.get_name())
    rospy.spin()

