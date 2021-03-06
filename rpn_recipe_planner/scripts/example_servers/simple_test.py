#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionServer
from rpn_recipe_planner_msgs.msg import SimpleTestAction, SimpleTestResult


class SimpleTestServer(object):
    def __init__(self, name):
        self._ps = SimpleActionServer(
            name,
            SimpleTestAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()

    def execute_cb(self, goal):
        print goal
        print "Got the value:", goal.value
        r = goal.value+1
        print "Increasing value by 1 and returning it as result"
        self._ps.set_succeeded(SimpleTestResult(r))

    def preempt_cb(self, *args):
        print "Nothing to preempt"


if __name__ == "__main__":
    rospy.init_node("test_server")
    s = SimpleTestServer(rospy.get_name())
    rospy.spin()
