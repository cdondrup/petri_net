#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from actionlib import SimpleActionServer
from rpn_recipe_planner_msgs.msg import SimpleTestAction, SimpleTestGoal
import json


class SimpleTestServer(object):
    def __init__(self, name):
        self._ps = SimpleActionServer(
            name,
            TestAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.run = True
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()

    def execute_cb(self, goal):
        print goal
        self._ps.set_succeeded()

    def preempt_cb(self, *args):
        pass


if __name__ == "__main__":
    rospy.init_node("test_server")
    s = SimpleTestServer(rospy.get_name())
    rospy.spin()
