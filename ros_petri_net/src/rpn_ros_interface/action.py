import rospy
import roslib
import rostopic
from actionlib import ActionClient
from actionlib_msgs.msg import GoalStatus
from threading import Lock, Thread
from pnp_kb.knowledgebase import KnowledgeBase
from pnp_actions.atomic_action import AtomicAction


class ROSAtomicAction(AtomicAction):
    def __init__(self, name, params=None, recovery=None):
        super(ROSAtomicAction, self).__init__(name, params, recovery)
        self.lock = Lock()
        self.g = None
        self.monitor_thread = None
        self.server_thread = None

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

    def call_server(self, kb):
        with self.lock:
            self.client = ActionClient(self.name, self.get_action_type(self.name))
            goal = self.get_goal_type(self.name)()
            for slot in set(goal.__slots__) - set(self.params.keys()):
                self.params[slot] = kb.query(slot)
            for slot, value in self.params.items():
                setattr(goal, slot, type(getattr(goal, slot))(value))
            self.client.wait_for_server()
            self.g = self.client.send_goal(goal)

    def start(self, kb):
        if self.server_thread is None or not self.server_thread.is_alive():
            self.server_thread = Thread(target=self.call_server, args=(kb,))
            self.server_thread.start()

    def wait_for_action(self, kb):
        if self.g is not None:
            with self.lock:
                while self.g.get_goal_status() in (GoalStatus.ACTIVE, GoalStatus.PENDING):
                    rospy.sleep(.01)
                result = self.g.get_result()
                if result != None and result:
                    for slot in result.__slots__:
                        res = getattr(result,slot)
                        kb.update(slot, res)

    def monitor(self, kb):
        if self.monitor_thread is None or not self.monitor_thread.is_alive():
            self.monitor_thread = Thread(target=self.wait_for_action, args=(kb,))
            self.monitor_thread.start()

    def get_state(self):
        if self.lock.acquire(False):
            try:
                return self.g.get_goal_status()
            except:
                return -1
            finally:
                self.lock.release()
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

