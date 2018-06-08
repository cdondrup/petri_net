import rospy
import numpy as np
from threading import Thread, Event
import uuid
from pprint import pprint


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
        monitor_threads = []
        action_finished = Event()
        while not rospy.is_shutdown():
            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            print "[{}] Current marking:".format(net_id), marking
            print "[{}] Current places:".format(net_id), net.get_current_places(marking)
            print "[{}] Monitored actions:".format(net_id)
            pprint(monitor_threads)
            # rospy.sleep(.1)
            # raw_input()
            print "[{}] Checking transitions.".format(net_id)
            trans = self.check_num_tokens(marking, net.d_minus)
            print "[{}] Transitions that should fire based on tokens:".format(net_id), trans
            pprint(net.get_current_transitions(trans))
            if np.sum(trans) == 0:
                break
            trans = self.check_conditions(trans, net.transitions)
            print "[{}] Transitions fiering based on condition:".format(net_id), trans
            pprint(net.get_current_transitions(trans))
            if len(net.get_current_places(marking)[0]) == len(monitor_threads) or np.sum(trans) == 0:
                action_finished.wait()
                action_finished.clear()
            print "[{}] Execute associated atomic_actions".format(net_id)
            self.execute_atomic_actions(trans, net.transitions, action_finished)
            marking = np.matmul(trans, net.d) + marking
            print "[{}] Monitoring atomic_actions".format(net_id)
            monitor_threads = self.monitor_atomic_actions(marking, net.places)
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

    def execute_atomic_actions(self, trans, transitions, event):
        for t in np.array(transitions)[trans == 1]:
            t.execute_atomic_action(event)

    def monitor_atomic_actions(self, marking, places):
        ts = []
        for p in np.array(places)[marking >= 1]:
            t = p.monitor_atomic_action()
            if t is not None:
                ts.append(t)
        return ts

