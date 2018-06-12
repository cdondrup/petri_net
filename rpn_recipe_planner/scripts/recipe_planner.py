#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from actionlib import ActionClient
from dialogue_arbiter_action.da_plugin_server import DAPluginServer
from rpn_recipe_planner_msgs.msg import RPNRecipePlannerAction
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
            self.plan["initial_knowledge"].update(params)
        print self.plan
        rpn = self.client.send_goal(
            RPNGoal(
                net_id=goal.id,
                domain=json.dumps(self.domain),
                plan=json.dumps(self.plan)
            ),
            transition_cb=self.trans_cb
        )

    def trans_cb(self, gh):
        if gh.get_goal_status() in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.ABORTED):
            gh.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("recipe_planner")
    r = RecipePlanner(rospy.get_name())
    rospy.spin()
