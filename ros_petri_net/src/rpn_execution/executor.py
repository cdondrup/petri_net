import rospy
import numpy as np
from threading import Thread
import uuid


class Executor(object):
    def __init__(self):
        self.nets = {}

    def add_net(self, net, marking):
        net_id = str(uuid.uuid4())
        self.nets[net_id] = {
            "net": net,
            "marking": marking,
            "thread": None
        }
        return net_id

    def execute_net(self, net_id):
        if self.nets[net_id]["thread"] is None or not self.nets[net_id]["thread"].is_alive():
            self.nets[net_id]["thread"] = Thread(target=self.__run, args=(net_id,))
            self.nets[net_id]["thread"].start()

    def __run(self, net_id):
        net = self.nets[net_id]["net"]
        marking = self.nets[net_id]["marking"]
        while not rospy.is_shutdown():
            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            print "[{}] Current marking:".format(net_id), marking
            print "[{}] Current places:".format(net_id), net.get_current_places(marking)
            rospy.sleep(.1)
            # raw_input()
            print "[{}] Checking transitions.".format(net_id)
            trans = self.check_num_tokens(marking, net.d_minus)
            print "[{}] Transitions that should fire based on tokens:".format(net_id), trans
            if np.sum(trans) == 0:
                break
            trans = self.check_conditions(trans, net.transitions)
            print "[{}] Transitions fiering based on condition:".format(net_id), trans
            print "[{}] Execute associated actions".format(net_id)
            self.execute_actions(trans, net.transitions)
            marking = np.matmul(trans, net.d) + marking
            print "[{}] Monitoring actions".format(net_id)
            self.monitor_actions(marking, net.places)
        if net.is_goal(marking):
            rospy.loginfo("Finished '{}' successfully.".format(net_id))
        elif net.is_fail(marking):
            rospy.loginfo("Finished '{}' unsuccessfully.".format(net_id))

    def check_num_tokens(self, marking, d_minus):
        trans = np.zeros(d_minus.shape[0], dtype=int)
        r = marking - d_minus
        trans[np.all(r >= 0, axis=1)] = 1
        return trans

    def check_conditions(self, trans, transitions):
        res = np.zeros(trans.shape[0], dtype=int)
        for idx, transition in enumerate(transitions):
            if trans[idx]:
                if transition.evaluate_condition():
                    res[idx] = 1
        return res

    def execute_actions(self, trans, transitions):
        for t in np.array(transitions)[trans == 1]:
            t.execute_action()

    def monitor_actions(self, marking, places):
        for p in np.array(places)[marking >= 1]:
            p.monitor_action()

