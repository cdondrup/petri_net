import rospy
import roslib
import rostopic
from actionlib import ActionClient, CommState
from actionlib_msgs.msg import GoalStatus
from threading import Thread, Event
from pnp_kb.knowledgebase import KnowledgeBase
from pnp_kb.queries import Query, LocalQuery, RemoteQuery
from pnp_kb.updates import Update, LocalUpdate, RemoteUpdate
from action import ROSAtomicAction
from copy import deepcopy
from ros_petri_net_msgs.srv import RPNQuery, RPNQueryResponse, RPNQueryRequest
from ros_petri_net_msgs.srv import RPNUpdate, RPNUpdateResponse, RPNUpdateRequest
import json


class RPNAtomicAction(ROSAtomicAction):
    def run(self, kb, external_kb):
        with self.__mutex__:
            server_finished = Event()

            def trans_cb(gh):
                state = gh.get_comm_state()
                goal_state = gh.get_goal_status()
                print "{}({}): comm changed state to: {}".format(self.name, ', '.join(self.params), state)
                print "{}({}): goal changed state to: {}".format(self.name, ', '.join(self.params), goal_state)
                if state == CommState.DONE:
                    print "SETTING server finished"
                    server_finished.set()
                    print "SET server finished"

            def query_cb(req):
                q_type = dict(zip((RPNQueryRequest.ALL, RPNQueryRequest.LOCAL, RPNQueryRequest.REMOTE), (Query, LocalQuery, RemoteQuery)))
                q = q_type[req.type](req.attr, req.meta_info)
                result = q(kb, external_kb)
                result = json.dumps(result) if not isinstance(result, (str, unicode)) else result
                return RPNQueryResponse(result)

            def update_cb(req):
                u_type = dict(zip((RPNUpdateRequest.ALL, RPNUpdateRequest.LOCAL, RPNUpdateRequest.REMOTE), (Update, LocalUpdate, RemoteUpdate)))
                u_type[req.type](req.attr, req.value, req.meta_info)(kb, external_kb)
                return RPNUpdateResponse()

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
                srv_basename = "/"+self.gh.comm_state_machine.action_goal.goal_id.id.replace('/','').replace('-','_').replace('.','_')
                q_srv = rospy.Service(srv_basename+"/query", RPNQuery, query_cb)
                u_srv = rospy.Service(srv_basename+"/update", RPNUpdate, update_cb)
                print "WAITING TO FINISH (RPN)"
                server_finished.wait()
                print "FINISHED (RPN)"
                self.get_result(kb)
                q_srv.shutdown()
                u_srv.shutdown()

    def __str__(self):
        return "RPNAction({}): [{}]".format(
            self.name,
            ', '.join([str(k)+": "+str(v) for k,v in self.params.items()]) \
                if self.params is not None else ""
        )

