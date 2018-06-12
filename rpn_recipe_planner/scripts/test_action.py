#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from dialogue_arbiter_action.da_simple_plugin_server import DASimplePluginServer
from dialogue_arbiter.msg import TestAction, TestGoal
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
import json


class TestServer(object):
    def __init__(self, name):
        self.ontology = {
            "a coffee shop": ["Costa", "Starbucks"],
            "coffee shop": ["Costa", "Starbucks"]
        }
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
        goal.place_frame = goal.place_frame.strip()
        if goal.place_frame in self.ontology:
            print "TEST SERVER", "disambiguate"
            # print "TEST SERVER", self._ps.query_controller(status="disambiguation", return_value=json.dumps(self.ontology[goal.place_frame]))
            rospy.sleep(1.)
        print "TEST SERVER", "clarification.landmark"
        # print "TEST SERVER", self._ps.query_controller(status="clarification.landmark", return_value="Zizzi")
        rospy.sleep(1.)
        print "TEST SERVER", "clarification.route_constraint"
        # print "TEST SERVER", self._ps.query_controller(status="clarification.route_constraint", return_value="Stairs")
        rospy.sleep(1.)
        print "TEST SERVER", "execute.route_description"
        # print "TEST SERVER", self._ps.query_controller(status="execute.route_description", return_value=json.dumps([["Starbucks", "right"], ["waypoint 1", "left"], ["waypoint 2", "straight"]]))
        rospy.sleep(1.)
        print "TEST SERVER", "ended"
        self._ps.set_succeeded()

    def preempt_cb(self, *args):
        self.run = False


if __name__ == "__main__":
    rospy.init_node("test_server")
    t = TestServer(rospy.get_name())
    rospy.spin()
