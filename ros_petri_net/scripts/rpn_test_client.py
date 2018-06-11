#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionClient
from petri_net_msgs.msg import RPNAction, RPNGoal
import yaml
import json


class TestClient(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}'...".format(name))
        self.client = SimpleActionClient("/RPN", RPNAction)
        self.client.wait_for_server()
        with open(rospy.get_param("~plan"), 'r') as f:
            self.domain, self.plan = [x for x in yaml.load_all(f)]
        self.client.send_goal_and_wait(RPNGoal(domain=json.dumps(self.domain), plan=json.dumps(self.plan)))
        rospy.loginfo("Started '{}'.".format(name))


if __name__ == "__main__":
    rospy.init_node("rpn_test_client")
    t = TestClient(rospy.get_name())
    # rospy.spin()

