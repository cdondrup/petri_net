#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from rpn_recipe_planner_msgs.msg import ExampleRouteGenerationAction, ExampleRouteGenerationResult
from rpn_recipe_planner_msgs.msg import SemanticRoute as SRmsg, SemanticRouteLocation
from rpn_action_servers.rpn_simple_action_server import RPNSimpleActionServer
from semantic_route_description.srv import SemanticRoute, SemanticRouteRequest
from ontologenius.srv import OntologeniusService, OntologeniusServiceRequest
import json
import numpy as np
from pprint import pprint


START_LOCATION = "pepper_infodesk"

class TestServer(object):
    def __init__(self, name):
        self._ps = RPNSimpleActionServer(
            name,
            ExampleRouteGenerationAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)
        self.cache = {}
        self._ps.start()
        self.__is_alive()

    def __is_alive(self):
        from threading import Thread
        from std_msgs.msg import String

        pub = rospy.Publisher("~is_alive", String, queue_size=1)

        def publish():
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                pub.publish(str(rospy.Time.now().to_sec()))
                r.sleep()

        t = Thread(target=publish)
        t.start()

    def execute_cb(self, goal):
        self.close_ontology()
        self.cache = {}
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
                                "name": self.__get_name(y),
                                "type": self.__get_type(y)
                            },
                            x.route
                        ),
                        res.routes)
                )
            )
            costs = np.array(res.costs)
            costs.sort()
            pprint(rd)

            r = ExampleRouteGenerationResult()

            for c in costs:
                print "#########"
                s = SRmsg()
                for p in rd[c]:
                    s.route.append(
                        SemanticRouteLocation(**p)
                    )
                r.route_array.append(s)

            self._ps.set_succeeded(r)

    def __get_direction(self, place, corridor, pplace):
        pp = self.__get_with(pplace, corridor)
        end = self.__is_at_edge_of_corridor(place, corridor)
        if end:
            if end == -1:
                if "isAtRightOfPath" in pp:
                    return "right"
                elif "isAtLeftOfPath" in pp:
                    return "left"
                else:
                    return "confused (end)"
            else:
                if "isAtRightOfPath" in pp:
                    return "left"
                elif "isAtLeftOfPath" in pp:
                    return "right"
                else:
                    return "confused (beginning)"
        else:
            p = self.__get_with(place, corridor)
            if set(p) == set(pp): # same side
                l = self.get_closest_landmark(pplace, corridor, place)
                if l is not None:
                    return l[0]
            else:
                try:
                    l = self.get_closest_landmark(self.__get_on(pplace, "hasInFront")[0], corridor, place)
                    if l is not None:
                        return l[0]
                except IndexError:
                    return "confused (no infront)"
            return "confused (side)"

    def __is_at_edge_of_corridor(self, place, corridor):
        r = self.__get_with(place, corridor)
        return -1 if "isAtEndEdgeOfPath" in r else 1 if "isAtBeginEdgeOfPath" in r else 0

    def get_closest_landmark(self, place, path, specific_place=None):
        if specific_place is not None:
            try:
                specific_place = specific_place["id"]
            except:
                pass

        (LEFT, RIGHT) = ("hastAtLeft", "hasAtRight")
        left = [x for x in self.__get_on(place, LEFT) if self.__get_with(x, path)]
        right = [x for x in self.__get_on(place, RIGHT) if self.__get_with(x, path)]

        print LEFT, left
        print RIGHT, right

        while left or right:
            if left:
                if self.__is_landmark(left[0]):
                    if specific_place is None or left[0] == specific_place:
                        return "right", self.__get_name(left[0])
                left = self.__get_on(left[0], LEFT)
            if right:
                if self.__is_landmark(right[0]):
                    if specific_place is None or right[0] == specific_place:
                        return "left", self.__get_name(right[0])
                right = self.__get_on(right[0], RIGHT)

        return None

    def __get_on(self, attr, relation, select=None):
         return self.__get_info("getOn", attr, relation, select)

    def __get_with(self, attr1, attr2):
        return self.__get_info("getWith", attr1, attr2)

    def __get_info(self, action, attr1, attr2=None, select=None):
        try:
            attr1 = attr1["id"]
        except:
            pass
        try:
            attr2 = attr2["id"]
        except:
            pass

        action = ("" if select is None else "select:")+action
        params = ("" if select is None else select+"=")+attr1+(":"+attr2 if attr2 is not None else "")
        key = "{}, {}".format(action, params)
        if key not in self.cache:
            res = self.call_onto_service(action, params)
            self.cache[key] = res
            print "individual({}) = ".format(key), res
        else:
            res = self.cache[key]
            print "(cached) individual({}) = ".format(key), res

        return res

    def __get_type(self, p):
        try:
            return p["type"]
        except:
            return self.__get_info("getUp", p)

    def __get_name(self, p):
        try:
            return p["name"]
        except:
            return self.__get_info("getName", p)[0]

    def __is_landmark(self, place):
        return self.__is_shop(place) or "toilets" in self.__get_type(place) or "signpost" in self.__get_type(place)

    def __is_corridor(self, path):
        return "corridor" in self.__get_type(path)

    def __is_openspace(self, path):
        return "openspace" in self.__get_type(path)

    def __is_shop(self, place):
        return "shop" in self.__get_type(place)

    def __is_intersection(self, place):
        return "pathIntersection" in self.__get_type(place)

    def __is_interface(self, place):
        return "interface" in self.__get_type(place)

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

    def close_ontology(self):
        return self.call_service(
            "/ontologenius/actions",
            OntologeniusService,
            OntologeniusServiceRequest(action="close", param="")
        )


if __name__ == "__main__":
    rospy.init_node("route_generation")
    t = TestServer(rospy.get_name())
    rospy.spin()
