#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from dialogue_arbiter_action.da_simple_plugin_server import DASimplePluginServer
from rpn_recipe_planner_msgs.msg import ExampleRouteGenerationAction, ExampleRouteGenerationResult
from rpn_recipe_planner_msgs.msg import StringArray
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
import json


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNSimpleActionServer(
            name,
            ExampleRouteGenerationAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.run = True
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()

    def execute_cb(self, goal):
        print goal
        print "started"
        rospy.sleep(1.)
        r = ExampleRouteGenerationResult(
            route_array=[
                StringArray(["waypoint1", "Stairs", "waypoint2", "waypoint3", goal.place_frame]),
                StringArray(["waypoint1", "Escalators", "waypoint2", "waypoint3", goal.place_frame]),
                StringArray(["waypoint1", "waypoint2", "waypoint3", goal.place_frame])
            ]
        )
        self._ps.set_succeeded(r)

    def preempt_cb(self, *args):
        self.run = False


if __name__ == "__main__":
    rospy.init_node("route_generation")
    t = TestServer(rospy.get_name())
    rospy.spin()
