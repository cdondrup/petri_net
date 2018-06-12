#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 20 13:42:25 2016

@author: Christian Dondrup
"""

import rospy
from actionlib_msgs.msg import GoalStatus
from ros_petri_net_msgs.msg import DummyAction, DummyResult
from ros_petri_net_msgs.srv import RPNQuery
from ros_petri_net_msgs.srv import RPNUpdate
from rpn_action_servers.rpn_action_server import RPNActionServer
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
from threading import Thread
from actionlib import ActionServer


class WaitServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self._as = RPNActionServer(name, DummyAction, goal_cb=self.goal_cb, cancel_cb=self.cancel_cb, auto_start=False)
        # self._as = RPNSimpleActionServer(name, DummyAction, execute_cb=self.simple_execute, auto_start=False)
        self._as.start()
        rospy.loginfo("Done.")

    def goal_cb(self, gh):
        t = Thread(target=self.execute, args=(gh,))
        t.start()

    def cancel_cb(self, gh):
        gh.set_canceled(DummyActionResult())

    def execute(self, gh):
        gh.set_accepted()
        goal = gh.get_goal()
        print "Setting spam to eggs"
        self._as.update_kb(gh, RPNActionServer.LOCAL, "spam", "eggs", "meta_info")
        print "Got this from KB:", self._as.query_kb(gh, RPNActionServer.ALL, "spam", "meta_info")
        print "Waiting for %s seconds" % goal.value
        end = rospy.Time.now().to_sec() + goal.value
        while rospy.Time.now().to_sec() < end and not rospy.is_shutdown() \
            and not gh.status_tracker.status.status == GoalStatus.PREEMPTED:
            rospy.sleep(0.1)
        if not gh.status_tracker.status.status == GoalStatus.PREEMPTED:
            print DummyResult(time=goal.value)
            gh.set_succeeded(DummyResult(time=goal.value))
            # gh.set_canceled(DummyResult(time=goal.value))
            # gh.set_aborted(DummyResult(time=goal.value))

    # Simple action server
    def simple_execute(self, gh):
        goal = gh
        print "Setting spam to eggs"
        self._as.update_kb(RPNActionServer.LOCAL, "spam", "eggs")
        print "Got this from KB:", self._as.query_kb(RPNActionServer.ALL, "spam")
        print "Waiting for %s seconds" % goal.value
        end = rospy.Time.now().to_sec() + goal.value
        while rospy.Time.now().to_sec() < end and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print DummyResult(time=goal.value)
        self._as.set_succeeded(DummyResult(time=goal.value))

if __name__ == "__main__":
    rospy.init_node("dummy_server")
    w = WaitServer(rospy.get_name())
    rospy.spin()

