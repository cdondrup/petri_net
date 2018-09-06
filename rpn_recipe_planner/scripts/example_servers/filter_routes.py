#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from rpn_action_servers.rpn_action_server import RPNActionServer
from rpn_recipe_planner_msgs.msg import FilterRoutesAction, FilterRoutesResult
import json
from copy import deepcopy
from threading import Thread


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNActionServer(
            name,
            FilterRoutesAction,
            self.goal_cb,
            auto_start=False
        )
        self.threads = {}
        self._ps.start()

    def goal_cb(self, gh):
        print "Goal handle", gh
        gh.set_accepted()
        goal = gh.get_goal()

        self.threads[gh] = Thread(target=self.execute, args=(gh, goal))
        self.threads[gh].start()

    def execute(self, gh, goal):
        print self._ps.get_goal_id(gh), "Goal", goal
        res = FilterRoutesResult()
        res.route = None
        confirmed_constraints = []
        for route in deepcopy(goal.route_array):
            good = True
            for l in route.route:
                for c in deepcopy(goal.route_constraints):
                    print "---"
                    print c
                    print l
                    if c in l.type:
                        if c in confirmed_constraints:
                            r = False
                        else:
                            print "QUERY"
                            qr = self._ps.query_kb(gh=gh, meta_info=json.dumps({"status": "clarification.route_constraint"}), type=RPNActionServer.QUERY_REMOTE, attr=l.name)
                            print self._ps.get_goal_id(gh), qr
                            r = json.loads(qr.value)
                        if r:
                            del goal.route_constraints[goal.route_constraints.index(c)]
                        else:
                            del goal.route_array[goal.route_array.index(route)]
                            if c not in confirmed_constraints:
                                confirmed_constraints.append(c)
                            good = False
                            break
                if not good:
                    break

            if good:
                res.route = route.route
                break

        try:
            if res.route is None:
                res.route = goal.route_array[-1].route
        except IndexError:
            self._ps.update_kb(gh=gh, meta_info=json.dumps({"status": "failed"}), type=RPNActionServer.UPDATE_REMOTE, attr="USER", value="")
            gh.set_aborted()
        else:
            print self._ps.get_goal_id(gh), type(res.route)
            res.route_array = goal.route_array
            res.route_constraints = goal.route_constraints
            print self._ps.get_goal_id(gh), "Res", res
            gh.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("filter_routes")
    t = TestServer(rospy.get_name())
    rospy.spin()
