#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from dialogue_arbiter_action.da_simple_plugin_server import DASimplePluginServer
from dialogue_arbiter.msg import TestAction, TestGoal
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
import json


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNSimpleActionServer(
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
        self.run = True
        print "started"
        rospy.sleep(1.)
        print "TEST SERVER", "clarification.landmark"
        print "TEST SERVER", self._ps.query_kb(meta_info=json.dumps({"status": "clarification.landmark"}), type=RPNSimpleActionServer.QUERY_REMOTE, attr="Zizzi")
        rospy.sleep(1.)
        print "TEST SERVER", "clarification.route_constraint"
        print "TEST SERVER", self._ps.query_kb(meta_info=json.dumps({"status": "clarification.route_constraint"}), type=RPNSimpleActionServer.QUERY_REMOTE, attr="Stairs")
        rospy.sleep(1.)
        print "TEST SERVER", "execute.route_description"
        print "TEST SERVER", self._ps.query_kb(meta_info=json.dumps({"status": "execute.route_description"}), type=RPNSimpleActionServer.QUERY_REMOTE, attr=json.dumps([["Starbucks", "right"], ["waypoint 1", "left"], ["waypoint 2", "straight"]]))
        rospy.sleep(1.)
        print "TEST SERVER", "ended"
        self._ps.set_succeeded()

    def preempt_cb(self, *args):
        self.run = False


if __name__ == "__main__":
    rospy.init_node("test_server")
    t = TestServer(rospy.get_name())
    rospy.spin()
