#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionServer
from rpn_action_servers.rpn_action_server import RPNActionServer
from rpn_recipe_planner_msgs.msg import SupervisionServerAction, SupervisionServerResult
from rpn_recipe_planner_msgs.msg import SupervisionServerInformAction, SupervisionServerInformGoal
from rpn_recipe_planner_msgs.msg import SupervisionServerQueryAction, SupervisionServerQueryGoal
# from guiding_as_msgs.msg import Task
from rpn_recipe_planner_msgs.srv import SuperQuery, SuperQueryResponse, SuperQueryRequest, SuperInform, SuperInformResponse, SuperInformRequest
from dialogue_arbiter.msg import DialogueArbiterActionResult
from threading import Thread
from collections import OrderedDict
from guiding_as_msgs.msg import taskGoal, taskAction, taskActionResult
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import json


class TestServer(object):
    def __init__(self, name):
        self._ps = SimpleActionServer(
            "/guiding_task",
            taskAction,
            self.goal_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)

        print "Waiting form inform server"
        self.inform_client = SimpleActionClient("/rpn_supervision_server/inform_srv", SupervisionServerInformAction)
        self.inform_client.wait_for_server()
        print "Waiting form query server"
        self.query_client = SimpleActionClient("/rpn_supervision_server/query_srv", SupervisionServerQueryAction)
        self.query_client.wait_for_server()

        print "starting"
        self._ps.start()

    def goal_cb(self, goal):
        print "received goal"
        print goal
        print "where are you"
        g = SupervisionServerInformGoal(status="verbalisation.where_are_u")
        self.inform_client.send_goal_and_wait(g)
        print "found you"
        g = SupervisionServerInformGoal(status="verbalisation.found_again")
        self.inform_client.send_goal_and_wait(g)
        s = rospy.ServiceProxy("/rpn_supervision_server/query", SuperQuery)
        print "Ask if seen"
        g = SupervisionServerQueryGoal(status="clarification.cannot_tell_seen")
        self.query_client.send_goal_and_wait(g)
        r = self.query_client.get_result()
        print "result:", r
        if not self._ps.is_preempt_requested():
            self._ps.set_succeeded()
        else:
            self._ps.set_preempted()

    def preempt_cb(self, *args):
        print "preempting"
        self.inform_client.cancel_all_goals()
        self.query_client.cancel_all_goals()


if __name__ == "__main__":
    rospy.init_node("dummy_supervision_server")
    t = TestServer(rospy.get_name())
    rospy.spin()
