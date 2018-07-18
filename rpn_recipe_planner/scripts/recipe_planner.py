#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from actionlib import ActionClient
from actionlib_msgs.msg import GoalStatus
from dialogue_arbiter_action.da_plugin_server import DAPluginServer
from rpn_recipe_planner_msgs.msg import RPNRecipePlannerAction
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryResponse
from rpn_recipe_planner_msgs.srv import RPInform, RPInformResponse
from ros_petri_net_msgs.msg import RPNAction, RPNGoal
import json
import yaml


class RecipePlanner(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}'...".format(name))
        worlds_dir = rospy.get_param("~worlds_dir")
        worlds_dir = worlds_dir if worlds_dir[-1] == "/" else worlds_dir + "/"
        worlds = self.load_yaml(worlds_dir+rospy.get_param("~worlds_file"))
        world = worlds[rospy.get_param("~world")]
        self.domain = self.load_yaml(worlds_dir+world["domain"])
        self.recipes = {}
        for r in (self.load_yaml(worlds_dir+x) for x in world["recipes"]):
            print r
            self.recipes.update(r)
        print self.domain
        print self.recipes
        self.servers = {k: Server(k, self.domain, v) for k, v in self.recipes.items()}
        print self.servers
        rospy.loginfo("Started '{}'.".format(name))

    def load_yaml(self, filename):
        with open(filename, 'r') as f:
            return yaml.load(f)


class Server(object):
    def __init__(self, name, domain, plan):
        self.name = name
        self.domain = domain
        self.plan = plan
        self.services = {}
        self.goal_handles = {}
        self.rpn = {}
        self._ps = DAPluginServer(
            name,
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
            rospy.Service("/"+goal.id.replace('-','_')+"/query", RPQuery, lambda x: self.query_cb(x, goal.id))
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
        if gh.get_goal_status() in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED):
            net_id = gh.comm_state_machine.action_goal.goal.net_id
            try:
                for s in self.services[net_id]:
                    s.shutdown()
                self.goal_handles[net_id].set_succeeded()
                del self.services[net_id]
                del self.goal_handles[net_id]
                del self.rpn[net_id]
            except KeyError:
                rospy.logwarn("No net with id '{}' currently active.".format(net_id))


    def query_cb(self, req, net_id):
        r = self._ps.query_controller(req.status, req.return_value, net_id)
        return RPQueryResponse(r.result)

if __name__ == "__main__":
    rospy.init_node("recipe_planner")
    r = RecipePlanner(rospy.get_name())
    rospy.spin()
