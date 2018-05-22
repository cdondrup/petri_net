#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
from rpn_common.transition import Transition
from rpn_common.place import Place


class PetriNetNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        p = Place('test')
        self.transitions = np.array([
            Transition('Transition_1', condition=True, action="Action_1"),
            Transition('Transition_2', condition=True),
            Transition('Transition_3', condition=True, action="Action_3"),
            Transition('Transition_4', condition=True)
        ])
        self.places = np.array([
            Place('Place_1', action='Action_1'),
            Place('Place_2', action='Action_2'),
            Place('Place_3'),
            Place('Place_4'),
            Place('Place_5'),
        ])
        self.d_minus = np.array([
            [0,0,0,0,1],
            [1,0,0,0,0],
            [0,1,0,0,0],
            [0,0,1,3,0]
        ])
        self.d_plus = np.array([
            [1,1,0,0,0],
            [0,0,2,1,0],
            [0,0,0,1,0],
            [0,0,0,0,4]
        ])
        self.d = self.d_plus - self.d_minus
        print "D:", self.d
        print "Enter initial marking (JSON):"
        marking = json.loads(raw_input())
        while not rospy.is_shutdown():
            print "Current marking:", marking
            # rospy.sleep(1.)
            raw_input()
            print "Checking transitions."
            trans = self.check_num_tokens(marking)
            print "Transitions that should fire based on tokens:", trans
            trans = self.check_conditions(trans)
            print "Transitions fiering based on condition:", trans
            print "Execute associated actions"
            self.execute_actions(trans)
            marking = np.matmul(trans, self.d) + marking
            print "Monitoring actions"
            self.monitor_actions(marking)
        rospy.loginfo("Started '{}'.".format(name))

    def check_num_tokens(self, marking):
        trans = np.zeros(self.d_minus.shape[0])
        r = marking - self.d_minus
        # print marking
        # print self.d_minus
        # print r
        trans[np.all(r >= 0, axis=1)] = 1
        return trans

    def check_conditions(self, trans):
        res = np.zeros(trans.shape[0])
        for idx, transition in enumerate(self.transitions):
            if trans[idx]:
                if transition.evaluate_condition():
                    res[idx] = 1
        return res

    def execute_actions(self, trans):
        for t in self.transitions[trans == 1]:
            t.execute_action()

    def monitor_actions(self, marking):
        for p in self.places[marking >= 1]:
            p.monitor_action()



if __name__ == "__main__":
    rospy.init_node("ros_petri_net_node")
    p = PetriNetNode(rospy.get_name())
    rospy.spin()

