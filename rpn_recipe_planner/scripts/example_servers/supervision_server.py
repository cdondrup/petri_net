#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionServer
from rpn_action_servers.rpn_action_server import RPNActionServer
from rpn_recipe_planner_msgs.msg import SupervisionServerAction, SupervisionServerResult
from rpn_recipe_planner_msgs.msg import SupervisionServerInformAction, SupervisionServerInformResult
from rpn_recipe_planner_msgs.msg import SupervisionServerQueryAction, SupervisionServerQueryResult
# from guiding_as_msgs.msg import Task
from rpn_recipe_planner_msgs.srv import SuperQuery, SuperQueryResponse, SuperQueryRequest, SuperInform, SuperInformResponse, SuperInformRequest
from dialogue_arbiter.msg import DialogueArbiterActionResult
from threading import Thread, Event
from collections import OrderedDict
from guiding_as_msgs.msg import taskGoal, taskAction, taskActionResult
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from queue import Queue
import json


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNActionServer(
            name,
            SupervisionServerAction,
            self.goal_cb,
            auto_start=False
        )

        self.inform_srv = SimpleActionServer("~inform_srv", SupervisionServerInformAction, lambda x: self.server_cb(x, "inform"), auto_start=False)
        self.query_srv = SimpleActionServer("~query_srv", SupervisionServerQueryAction, lambda x: self.server_cb(x, "query"), auto_start=False)

        self.threads = OrderedDict({})
        # self.pub = rospy.Publisher("/supervisor/new_goal", Task, queue_size=10)
        # self.sub = rospy.Subscriber("/guiding_task/result", taskActionResult, self.result_cb)
        self.srv = rospy.Service("~inform", SuperInform, lambda x: self.srv_cb(x, "inform"))
        self.srv = rospy.Service("~query", SuperQuery, lambda x: self.srv_cb(x, "query"))
        self.inform_srv.start()
        self.query_srv.start()
        self._ps.start()
        self.__is_alive()

    def __is_alive(self):
        from threading import Thread
        from std_msgs.msg import String

        pub = rospy.Publisher("~is_alive", String, queue_size=1)

        def publish():
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                pub.publish(str(rospy.Time.now().to_sec()))
                r.sleep()

        t = Thread(target=publish)
        t.start()

    # def result_cb(self, msg):
        # print "RESULT MSG", msg
        # gh = self.threads.items()[-1][0]
        # if msg.result.success:
            # gh.set_succeeded()
        # else:
            # gh.set_aborted()
        # print self.threads
        # del self.threads[gh]
        # print self.threads

    def goal_cb(self, gh):
        print "received goal"
        gh.set_accepted()
        goal = gh.get_goal()
        print goal
        self.threads[gh] = Thread(target=self.execute, args=(gh, goal))
        # print self.threads
        self.threads[gh].start()

    def execute(self, gh, goal):
        sg = taskGoal(place_frame=goal.place_frame, person_frame=goal.person_frame)
        client = SimpleActionClient("/guiding_task", taskAction)
        print "Waiting for super vision server"
        client.wait_for_server()
        print "Found super vision server"
        print "Sending goal and waiting for it to finish"
        client.send_goal_and_wait(sg)
        print "Supervision server finished"
        status = client.get_state()
        print "STATUS", status
        if status == GoalStatus.SUCCEEDED:
            gh.set_succeeded()
            return
        elif status in (GoalStatus.PREEMPTED, GoalStatus.PREEMPTING):
            gh.set_canceled()
            return
        gh.set_aborted()

    def server_cb(self, goal, type):
        event = Event()
        queue = Queue()

        def call_service(req, type):
            queue.put(self.srv_cb(req, type))

        print "Received new {} goal:".format(type)
        print goal
        if type == "inform":
            print "Called inform action server"
            t = Thread(target=call_service, args=(SuperInformRequest(status=goal.status, return_value=goal.return_value), type))
            t.start()
            while not rospy.is_shutdown() and not self.inform_srv.is_preempt_requested() and t.is_alive():
                rospy.sleep(0.1) # Ugly busy wait but don't know any better
            if not self.inform_srv.is_preempt_requested() and self.inform_srv.is_active():
                self.inform_srv.set_succeeded()
            else:
                self.query_srv.set_preempted()
        else:
            print "Called query action server"
            t = Thread(target=call_service, args=(SuperQueryRequest(status=goal.status, return_value=goal.return_value), type))
            t.start()
            while not rospy.is_shutdown() and not self.query_srv.is_preempt_requested() and t.is_alive():
                rospy.sleep(0.1) # Ugly busy wait but don't know any better
            if not t.is_alive():
                r = queue.get()
            if not self.query_srv.is_preempt_requested() and self.query_srv.is_active():
                self.query_srv.set_succeeded(r)
            else:
                self.query_srv.set_preempted()

    def srv_cb(self, req, type):
        print "Called {} service:".format(type)
        print req
        gh = self.threads.items()[-1][0]
        if type == "inform":
            print "Sending inform to Alana"
            self._ps.update_kb(gh=gh, meta_info=json.dumps({"status": req.status}), type=RPNActionServer.UPDATE_REMOTE, value=json.dumps(req.return_value), attr="USER")
            print "success"
            return SuperInformResponse()
        else:
            print "Sending query to Alana"
            r = self._ps.query_kb(gh=gh, meta_info=json.dumps({"status": req.status}), type=RPNActionServer.QUERY_REMOTE, attr=json.dumps(req.return_value)).value
            print "success:", r
            return SuperQueryResponse(r)


if __name__ == "__main__":
    rospy.init_node("rpn_supervision_server")
    t = TestServer(rospy.get_name())
    rospy.spin()
