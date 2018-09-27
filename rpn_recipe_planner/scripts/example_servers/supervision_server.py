#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rpn_action_servers.rpn_action_server import RPNActionServer
from rpn_recipe_planner_msgs.msg import SupervisionServerAction, SupervisionServerResult
from guiding_as_msgs.msg import Task
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryResponse, RPInform, RPInformResponse
from dialogue_arbiter.msg import DialogueArbiterActionResult
from threading import Thread
from collections import OrderedDict


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNActionServer(
            name,
            SupervisionServerAction,
            self.goal_cb,
            auto_start=False
        )
        self.threads = OrderedDict({})
        self.pub = rospy.Publisher("/supervisor/new_goal", Task, queue_size=10)
        self.sub = rospy.Subscriber("/new_task_route_descr/result", DialogueArbiterActionResult, self.result_cb)
        self.srv = rospy.Service("~inform", RPInform, lambda x: self.srv_cb(x, "inform"))
        self.srv = rospy.Service("~query", RPQuery, lambda x: self.srv_cb(x, "query"))
        self._ps.start()

    def result_cb(self, msg):
        gh = self.threads.items()[-1][0]
        gh.set_succeeded()
        print self.threads
        del self.threads[gh]
        print self.threads

    def goal_cb(self, gh):
        print "received goal"
        gh.set_accepted()
        goal = gh.get_goal()
        print goal
        self.threads[gh] = Thread(target=self.execute, args=(gh, goal))
        print self.threads
        self.threads[gh].start()

    def execute(self, gh, goal):
        t = Task(place_frame=goal.place_frame, person_frame=goal.person_frame)
        print "sending message"
        self.pub.publish(t)

    def srv_cb(self, req, type):
        gh = self.threads.items()[-1][0]
        if type == "inform":
            self._ps.query_kb(gh=gh, meta_info=json.dumps({"status": req.status}), type=RPNActionServer.UPDATE_REMOTE, value=json.dumps(req.return_value), attr="USER")
            return RPInformResponse()
        else:
            r = self._ps.query_kb(gh=gh, meta_info=json.dumps({"status": req.status}), type=RPNActionServer.QUERY_REMOTE, attr=json.dumps(req.return_value)).value
            return RPQueryResponse(r)


if __name__ == "__main__":
    rospy.init_node("rpn_supervision_server")
    t = TestServer(rospy.get_name())
    rospy.spin()
