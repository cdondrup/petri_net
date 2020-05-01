#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from rpn_recipe_planner_msgs.msg import SimpleTestAction, SimpleTestGoal
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNSimpleActionServer(
            name,
            SimpleTestAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.run = True
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()

    def execute_cb(self, goal):
        print goal
        self.run = True
        print "started"
        rospy.sleep(1.)
        print "TEST SERVER", "Saving value to external KB"
        print "TEST SERVER", self._ps.update_kb(type=RPNSimpleActionServer.QUERY_REMOTE, attr="my_value", value=goal.value)
        rospy.sleep(1.)
        print "TEST SERVER", "Getting value from external KB"
        print "TEST SERVER", self._ps.query_kb(type=RPNSimpleActionServer.QUERY_REMOTE, attr="my_value")
        rospy.sleep(1.)
        print "TEST SERVER", "ended"
        #  Passing the value of the goal to the result
        self._ps.set_succeeded(goal.value)

    def preempt_cb(self, *args):
        self.run = False


if __name__ == "__main__":
    rospy.init_node("rpn_test_server")
    t = TestServer(rospy.get_name())
    rospy.spin()
