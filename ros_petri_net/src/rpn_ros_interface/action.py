import rospy
import roslib
import rostopic
from actionlib import ActionClient
from actionlib_msgs.msg import GoalStatus
from threading import Thread, Event
from pnp_kb.knowledgebase import KnowledgeBase
from pnp_actions.atomic_action import AtomicAction
from copy import deepcopy
from StringIO import StringIO
from genpy import SerializationError


class ROSAtomicAction(AtomicAction):
    def __init__(self, name, params=None):
        super(ROSAtomicAction, self).__init__(name, params)
        self.error = False

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
                    server_finished.set()

            action_type = self.get_action_type(self.name)
            goal_type = self.get_goal_type(self.name)
            try:
                goal = self.create_goal(goal_type=goal_type, params=self.params, kb=kb)
            except SerializationError as e:
                pass
            else:
                print goal
                self.client = ActionClient(self.name, action_type)
                wr = self.client.wait_for_server(timeout=rospy.Duration(1.0))
                if not wr:
                    self.error = True
                    print "{}({}): failed, couldn't find a server with that name.".format(self.name, ', '.join(self.params))
                    return
                self.gh = self.client.send_goal(goal, transition_cb=trans_cb)
                server_finished.wait()
                self.get_result(kb)

    def get_result(self, kb):
        result = self.gh.get_result()
        if result != None and result:
            for slot in result.__slots__:
                res = getattr(result,slot)
                kb.update(slot, res)

    def create_goal(self, goal_type, params, kb):
        self.error = False
        goal = goal_type()
        tmp = deepcopy(params)  # Prevent to save the current state of non-fixed params
        for slot in set(goal.__slots__) - set(tmp.keys()):
            tmp[slot] = kb.query(slot)
        for slot, value in tmp.items():
            setattr(goal, slot, value)
        try:
            rospy.msg.serialize_message(b=StringIO(), msg=goal, seq=1)
        except SerializationError as e:
            print "{}({}): Goal serialisation failed, trying to forcefully cast parameters to match goal.".format(self.name, ', '.join(self.params))
            goal = goal_type()
            for slot, value in tmp.items():
                setattr(goal, slot, type(getattr(goal, slot))(value))
            try:
                rospy.msg.serialize_message(b=StringIO(), msg=goal, seq=1)
            except SerializationError as e:
                print "{}({}): Goal serialisation failed after casting parameters: {}".format(self.name, ', '.join(self.params, e))
                print "{}({}): Please, make sure that parameters with the same name also have the same type.".format(self.name, ', '.join(self.params))
                print "{}({}): Automatic parameter population failed, aborting action.".format(self.name, ', '.join(self.params))
                self.error = True
                raise e

        return goal

    def get_state(self):
        if self.__mutex__.acquire(False):
            try:
                if self.error:
                    return GoalStatus.LOST
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

    # def get_result(self):
        # if self.g is not None:
            # with self.lock:
                # return self.g.get_result()

    def __str__(self):
        return "ROSAction({}): [{}]".format(
            self.name,
            ', '.join([str(k)+": "+str(v) for k,v in self.params.items()]) \
                if self.params is not None else ""
        )

