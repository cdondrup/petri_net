import rospy
import roslib
import rostopic
from actionlib import ActionClient
from actionlib_msgs.msg import GoalStatus
from threading import Thread, Event
from pnp_kb.knowledgebase import KnowledgeBase
from action import ROSAtomicAction
from copy import deepcopy


class PNAtomicAction(ROSAtomicAction):
    def get_goal_type(self, action_name):
        topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
        # remove "Action" string from goal type
        assert("Action" in topic_type)
        return roslib.message.get_message_class(topic_type[:-10]+"Goal")

    def get_action_type(self, action_name):
        topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
        # remove "Goal" string from action type
        assert("Goal" in topic_type)
        return roslib.message.get_message_class(topic_type[:-4])

    def run(self, kb, external_kb):
        with self.__mutex__:
            server_finished = Event()

            def trans_cb(gh):
                print "{}({}): changed state to: {}".format(self.name, ', '.join(self.params), gh.get_goal_status())
                if gh.get_goal_status() in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED):
                    result = gh.get_result()
                    if result != None and result:
                        for slot in result.__slots__:
                            res = getattr(result,slot)
                            kb.update(slot, res)
                    server_finished.set()

            self.client = ActionClient(self.name, self.get_action_type(self.name))
            goal = self.get_goal_type(self.name)()
            tmp = deepcopy(self.params)  # Prevent to save the current state of non-fixed params
            for slot in set(goal.__slots__) - set(tmp.keys()):
                tmp[slot] = kb.query(slot)
            for slot, value in tmp.items():
                setattr(goal, slot, type(getattr(goal, slot))(value))
            self.client.wait_for_server()
            self.gh = self.client.send_goal(goal, transition_cb=trans_cb)
            server_finished.wait()
            result = self.gh.get_result()
            if result != None and result:
                for slot in result.__slots__:
                    res = getattr(result,slot)
                    kb.update(slot, res)

    def get_state(self):
        if self.__mutex__.acquire(False):
            try:
                return self.gh.get_goal_status()
            except:
                return -1
            finally:
                self.__mutex__.release()
        return -1

    @property
    def succeeded(self):
        return self.get_state() == GoalStatus.SUCCEEDED

    @property
    def preempted(self):
        return self.get_state() in (GoalStatus.PREEMPTED, GoalStatus.PREEMPTING,
                                    GoalStatus.RECALLED, GoalStatus.RECALLING)

    @property
    def failed(self):
        return self.get_state() in (GoalStatus.LOST, GoalStatus.ABORTED)

    def get_result(self):
        if self.g is not None:
            with self.lock:
                return self.g.get_result()

    def __str__(self):
        return "Action({}): [{}]".format(
            self.name,
            ', '.join([str(k)+": "+str(v) for k,v in self.params.items()]) \
                if self.params is not None else ""
        )

