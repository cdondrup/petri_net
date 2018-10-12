#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from actionlib import ActionClient, ActionServer
from actionlib_msgs.msg import GoalStatus
from dialogue_arbiter_action.da_plugin_server import DAPluginServer
from rpn_recipe_planner_msgs.msg import RPNRecipePlannerAction
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryResponse
from rpn_recipe_planner_msgs.srv import RPInform, RPInformResponse
from ros_petri_net_msgs.msg import RPNAction, RPNGoal
from dialogue_arbiter.msg import DialogueArbiterAction
import json
import yaml


class RecipePlanner(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}'...".format(name))
        da_action = rospy.get_param("~da_action", True)
        print da_action
        worlds_dir = rospy.get_param("~worlds_dir")
        worlds_dir = worlds_dir if worlds_dir[-1] == "/" else worlds_dir + "/"
        worlds = self.load_yaml(worlds_dir+rospy.get_param("~worlds_file"))
        world = worlds[rospy.get_param("~world")]
        print world
        self.domain = self.load_yaml(worlds_dir+world["domain"])
        self.recipes = {}
        for r in (self.load_yaml(worlds_dir+x) for x in world["recipes"]):
            print r
            self.recipes.update(r)
        print self.domain
        print self.recipes
        self.servers = {k: Server(k, self.domain, v, DAPluginServer if da_action else ActionServer) for k, v in self.recipes.items()}
        print self.servers
        rospy.loginfo("Started '{}'.".format(name))

    def load_yaml(self, filename):
        with open(filename, 'r') as f:
            return yaml.load(f)


class Server(object):
    def __init__(self, name, domain, plan, server_type):
        self.name = name
        self.domain = domain
        self.plan = plan
        self.services = {}
        self.goal_handles = {}
        self.rpn = {}
        self._ps = server_type(
            name,
            ActionSpec=DialogueArbiterAction,
            goal_cb=self.goal_cb,
            auto_start=False
        )
        self.client = ActionClient("/RPN", RPNAction)
        self.client.wait_for_server()
        self._ps.start()

    def goal_cb(self, gh):
        gh.set_accepted()
        goal = gh.get_goal()
        print goal
        if goal.params != '':
            params = json.loads(goal.params)
            for k in params.keys():
                try:
                    params[k] = params[k].strip()
                except:
                    pass
            self.plan["initial_knowledge"].update(params)
        print self.plan
        self.services[goal.id] = [
            rospy.Service("/"+goal.id.replace('-','_')+"/query", RPQuery, lambda x: self.query_cb(x, goal.id)),
            rospy.Service("/"+goal.id.replace('-','_')+"/inform", RPInform, lambda x: self.inform_cb(x, goal.id))
        ]
        self.goal_handles[goal.id] = gh
        self.rpn[goal.id] = self.client.send_goal(
            RPNGoal(
                net_id=goal.id,
                domain=json.dumps(self.domain),
                plan=json.dumps(self.plan)
            ),
            transition_cb=self.trans_cb
        )

    def trans_cb(self, gh):
        print self.services
        status = gh.get_goal_status()
        print "STATUS", status
        if status in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED):
            net_id = gh.comm_state_machine.action_goal.goal.net_id
            try:
                for s in self.services[net_id]:
                    s.shutdown()
                if status == GoalStatus.SUCCEEDED:
                    self.goal_handles[net_id].set_succeeded()
                else:
                    self.goal_handles[net_id].set_aborted()
                del self.services[net_id]
                del self.goal_handles[net_id]
                del self.rpn[net_id]
            except KeyError:
                rospy.logwarn("No net with id '{}' currently active.".format(net_id))

    def query_cb(self, req, net_id):
        if isinstance(self._ps, DAPluginServer):
            r = self._ps.query_controller(req.status, req.return_value, net_id)
            return RPQueryResponse(r.result)
        else:
            raise TypeError("Only DAPluginServers support querying.")

    def inform_cb(self, req, net_id):
        if isinstance(self._ps, DAPluginServer):
            self._ps.inform_controller(req.status, req.return_value, net_id)
            return RPInformResponse()
        else:
            raise TypeError("Only DAPluginServers support updating.")

if __name__ == "__main__":
    rospy.init_node("recipe_planner")
    r = RecipePlanner(rospy.get_name())
    rospy.spin()

