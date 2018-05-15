#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json


class PetriNetNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
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
            trans = np.zeros(self.d_minus.shape[0])
            r = marking - self.d_minus
            print "R:", r
            print r >= 0
            trans[np.where(np.all(r >= 0, axis=1))[0]] = 1
            print "Transitions fiering:", trans
            marking = np.matmul(trans, self.d) + marking
        rospy.loginfo("Started '{}'.".format(name))


if __name__ == "__main__":
    rospy.init_node("ros_petri_net_node")
    p = PetriNetNode(rospy.get_name())
    rospy.spin()

