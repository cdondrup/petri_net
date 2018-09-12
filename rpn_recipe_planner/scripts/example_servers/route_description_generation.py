#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from rpn_recipe_planner_msgs.msg import ExampleRouteDescriptionGenerationAction, ExampleRouteDescriptionGenerationResult
from rpn_action_servers.rpn_action_server import RPNActionServer
from semantic_route_description.srv import SemanticRoute, SemanticRouteRequest
from ontologenius.srv import OntologeniusService, OntologeniusServiceRequest
import json
import numpy as np
from pprint import pprint
from threading import Thread
import sys


class TestServer(object):
    def __init__(self, name):
        self._ps = RPNActionServer(
            name,
            ExampleRouteDescriptionGenerationAction,
            self.goal_cb,
            auto_start=False
        )
        # self._ps.register_preempt_callback(self.preempt_cb)
        self.threads = {}
        self.cache = {}
        self._ps.start()

    def goal_cb(self, gh):
        gh.set_accepted()
        goal = gh.get_goal()
        self.threads[gh] = Thread(target=self.execute, args=(gh, goal))
        self.threads[gh].start()

    def execute(self, gh, goal):
        self.close_ontology()
        self.cache = {}
        print goal
        route = goal.route.route
        route = map(lambda x: {"id": x.id, "name": x.name, "type": x.type}, route)
        print "started"

        route_descr = []

        route_list = zip(route[:-2:2], route[1:-1:2], route[2::2])
        for idx, (source, path, target) in enumerate(route_list):
            print idx
            print "src", source
            print "pat", path
            print "trg", target
            print "~~~"
            cl = self.get_closest_landmark(target, path)
            print "CLM", cl
            if self.__is_corridor(path):
                end = self.__is_at_edge_of_corridor(target, path)
                target["end"] = end
                print "END", end
                dir_ = self.__get_direction(target, path, source)
                print "DIR", dir_
                route_descr.append(
                    {"motion": {
                        "area": "the corridor",
                        "direction": dir_,
                        "theme": "you",
                        "source": ("the " if self.__requires_article(source) else "") + self.__get_name(source)
                    }}
                )
                if not self.__is_interface(source):
                    try:
                        route_descr[-1]["motion"]["distance"] = "at the end of" if source["end"] else "along"
                    except KeyError:
                        route_descr[-1]["motion"]["distance"] = "along"
                else:
                    route_descr[-1]["motion"]["distance"] = "along"
                if idx == len(route_list)-1:
                    route_descr.append(
                        {"being_located": {
                            "location": "at the end of the corridor" if end else ("along the corridor on the " + self.__get_side(target, path, source)),
                            "theme": self.__get_name(target),
                        }}
                    )
                    if cl is not None:
                        route_descr[-1]["being_located"]["place"] = cl[0] + (" of the " if self.__requires_article(cl[2]) else " of ") + cl[1]
                    else:
                        route_descr[-1]["being_located"]["place"] = ""
            else:
                if cl is not None: # and self.__is_interface(target):
                    route_descr.append(
                        {"taking": {
                            "agent": "you",
                            "source": cl[0] + (" of the " if self.__requires_article(cl[2]) else " of ") + cl[1],
                            "theme": ("the " if self.__requires_article(target) else "") + self.__get_name(target)
                        }}
                    )
            print "---"
        print route_descr
        qr = json.loads(self._ps.query_kb(gh=gh, meta_info=json.dumps({"status": "execute.route_description"}), type=RPNActionServer.QUERY_REMOTE, attr=json.dumps(route_descr)).value)
        print "QR", qr, type(qr)

        r = ExampleRouteDescriptionGenerationResult(qr)
        gh.set_succeeded(r)

    def __who_am_i(self):
        return sys._getframe(1).f_code.co_name

    def __get_side(self, place, corridor, pplace):
        print self.__who_am_i()
        # TODO: This still can't work due to missing empty place connectors. Needs fixing once onto has been updated.
        p = self.__get_with(place, corridor)
        pp = self.__get_with(pplace, corridor)
        if set(p) == set(pp): # same side
            l = self.get_closest_landmark(pplace, corridor, place)
            print "same side", l
            if l is not None:
                return l[0]
        else:
            try:
                l = self.get_closest_landmark(self.__get_on(pplace, "hasInFront")[0], corridor, place)
                print "other side", l
                if l is not None:
                    return l[0]
                else:
                    end = self.__is_at_edge_of_corridor(pplace, corridor)
                    if end:
                        return "left" if (end == -1 and "isAtLeftOfPath" in p) or (end == 1 and "isAtisAtRightOfPath" in p) else "right"
            except IndexError:
                return "confused (no infront)"
        return "straight"

    def __requires_article(self, place):
        print self.__who_am_i()
        return "toilets" in self.__get_type(place) or "signpost" in self.__get_type(place) or "atm" in self.__get_type(place) or self.__is_interface(place) or "pathIntersection" in self.__get_type(place)

    def __get_direction(self, place, corridor, pplace):
        print self.__who_am_i()
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
            return self.__get_side(place, corridor, pplace)

    def __is_at_edge_of_corridor(self, place, corridor):
        print self.__who_am_i()
        r = self.__get_with(place, corridor)
        return -1 if "isAtEndEdgeOfPath" in r else 1 if "isAtBeginEdgeOfPath" in r else 0

    def get_closest_landmark(self, place, path, specific_place=None):
        print self.__who_am_i()
        if specific_place is not None:
            try:
                specific_place = specific_place["id"]
            except:
                pass

        (LEFT, RIGHT) = ("hasAtLeft", "hasAtRight")
        left = [x for x in self.__get_on(place, LEFT) if self.__get_with(x, path)]
        right = [x for x in self.__get_on(place, RIGHT) if self.__get_with(x, path)]

        print LEFT, left
        print RIGHT, right

        while left or right:
            if left:
                if self.__is_landmark(left[0]):
                    if specific_place is None or left[0] == specific_place:
                        return "left", self.__get_name(left[0]), left[0]
                left = self.__get_on(left[0], LEFT)
            if right:
                if self.__is_landmark(right[0]):
                    if specific_place is None or right[0] == specific_place:
                        return "right", self.__get_name(right[0]), right[0]
                right = self.__get_on(right[0], RIGHT)

        return None

    def __get_on(self, attr, relation, select=None):
        print self.__who_am_i()
        return self.__get_info("getOn", attr, relation, select)

    def __get_with(self, attr1, attr2):
        print self.__who_am_i()
        return self.__get_info("getWith", attr1, attr2)

    def __get_info(self, action, attr1, attr2=None, select=None):
        print self.__who_am_i()
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
        print self.__who_am_i()
        try:
            return p["type"]
        except:
            return self.__get_info("getUp", p)

    def __get_name(self, p):
        print self.__who_am_i()
        try:
            name = p["name"]
        except:
            name = self.__get_info("getName", p)[0]
        if name.find("toilet") >= 0:
            return "toilet sign"
        return name

    def __is_landmark(self, place):
        print self.__who_am_i()
        return self.__is_shop(place) or "toilets" in self.__get_type(place) or "signpost" in self.__get_type(place)

    def __is_corridor(self, path):
        print self.__who_am_i()
        return "corridor" in self.__get_type(path)

    def __is_openspace(self, path):
        print self.__who_am_i()
        return "openspace" in self.__get_type(path)

    def __is_shop(self, place):
        print self.__who_am_i()
        return "shop" in self.__get_type(place)

    def __is_intersection(self, place):
        print self.__who_am_i()
        return "pathIntersection" in self.__get_type(place)

    def __is_interface(self, place):
        print self.__who_am_i()
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
    rospy.init_node("route_description_generation")
    t = TestServer(rospy.get_name())
    rospy.spin()
