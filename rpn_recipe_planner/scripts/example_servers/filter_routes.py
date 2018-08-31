#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
from rpn_recipe_planner_msgs.msg import FilterRoutesAction, FilterRoutesResult
import json
from copy import deepcopy


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNSimpleActionServer(
            name,
            FilterRoutesAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()

    def execute_cb(self, goal):
        print "Goal", goal
        # self._ps.update_kb(meta_info=json.dumps({"status": "verbalisation.robot_move"}), type=RPNSimpleActionServer.UPDATE_REMOTE, attr="USER", value="")
        res = FilterRoutesResult()
        res.route = None
        for route in deepcopy(goal.route_array):
            good = True
            for l in route.route:
                for c in deepcopy(goal.route_constraints):
                    if c in l.type:
                        qr = self._ps.query_kb(meta_info=json.dumps({"status": "clarification.route_constraint"}), type=RPNSimpleActionServer.QUERY_REMOTE, attr=l.name)
                        print qr
                        if json.loads(qr.value):
                            del goal.route_constraints[goal.route_constraints.index(c)]
                        else:
                            del goal.route_array[goal.route_array.index(route)]
                            good = False
                            break
                if not good:
                    break

            if good:
                res.route = route.route
                break

        if res.route is None:
            res.route = goal.route_array[-1].route

        print type(res.route)
        res.route_array = goal.route_array
        res.route_constraints = goal.route_constraints
        print "Res", res
        self._ps.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("filter_routes")
    t = TestServer(rospy.get_name())
    rospy.spin()
