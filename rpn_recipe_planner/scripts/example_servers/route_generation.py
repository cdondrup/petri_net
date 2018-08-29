#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from rpn_recipe_planner_msgs.msg import ExampleRouteGenerationAction, ExampleRouteGenerationResult
from rpn_recipe_planner_msgs.msg import StringArray
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
from semantic_route_description.srv import SemanticRoute, SemanticRouteRequest
from ontologenius.srv import OntologeniusService, OntologeniusServiceRequest
import json
import numpy as np
from pprint import pprint


START_LOCATION = "robot_infodesk"

class TestServer(object):
    def __init__(self, name):
        self._ps = RPNSimpleActionServer(
            name,
            ExampleRouteGenerationAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()

    def execute_cb(self, goal):
        print goal
        print "started"
        try:
            req = SemanticRouteRequest(
                from_=START_LOCATION,
                to=self.call_onto_service(
                    action="find",
                    param=goal.place_frame
                )[0],
                persona="lambda"
            )
        except IndexError:
            self._ps.set_aborted()
        else:
            res = self.call_route_service(req)
            rd = dict(
                zip(
                    res.costs,
                    map(
                        lambda x: map(
                            lambda y: {
                                "id": y,
                                "name": self.call_onto_service("getName", y)[0],
                                "type": self.call_onto_service("getUp", y)
                            },
                            x.route
                        ),
                        res.routes)
                )
            )
            costs = np.array(res.costs)
            costs.sort()
            pprint(rd)

            for c in costs:
                rd[c].reverse()
                print "#########"
                for e1, e2, e3 in zip(rd[c][:-2:2], rd[c][1:-1:2], rd[c][2::2]):
                    print e1
                    print e2
                    print e3
                    print "---"

            r = ExampleRouteGenerationResult(
                route_array=[
                    StringArray(["waypoint1", "Stairs", "waypoint2", "waypoint3", goal.place_frame]),
                    StringArray(["waypoint1", "Escalators", "waypoint2", "waypoint3", goal.place_frame]),
                    StringArray(["waypoint1", "waypoint2", "waypoint3", goal.place_frame])
                ]
            )
            self._ps.set_succeeded(r)

    def preempt_cb(self, *args):
        pass

    def call_onto_service(self, action, param):
        return self.call_service(
            "/ontologenius/individual",
            OntologeniusService,
            OntologeniusServiceRequest(action=action, param=param)
        ).values


    def call_route_service(self, req):
        return self.call_service(
            "/semantic_route_description/get_route",
            SemanticRoute,
            req
        )

    def call_service(self, srv_name, srv_type, req):
        while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    srv_name,
                    srv_type
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
                rospy.sleep(1.)
            else:
                return s(req)

if __name__ == "__main__":
    rospy.init_node("route_generation")
    t = TestServer(rospy.get_name())
    rospy.spin()
