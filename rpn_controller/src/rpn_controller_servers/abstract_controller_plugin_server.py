# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import actionlib
import json
import utils as ut
from abc import ABCMeta, abstractmethod, abstractproperty
from rpn_recipe_planner_msgs.msg import RosServerAction
from rpn_controller_msgs.srv import ControllerQuery, ControllerQueryRequest
from rpn_controller_msgs.srv import ControllerUpdate, ControllerUpdateRequest


class AbstractControllerPluginServer(actionlib.ActionServer, object):
    __metaclass__ = ABCMeta

    def __init__(self, ns, ActionSpec=RosServerAction, goal_cb=None, cancel_cb=actionlib.nop_cb, auto_start=True):
        if goal_cb is None:
            raise AttributeError("goal_cb has to be specified")
        self.ns = ns
        super(AbstractControllerPluginServer, self).__init__(
            ns=ns,
            ActionSpec=ActionSpec,
            goal_cb=goal_cb,
            cancel_cb=cancel_cb,
            auto_start=auto_start
        )

    @abstractproperty
    def controller_name(self):
        return

    def get_controller_name(self):
        if self.controller_name == "": return self.controller_name
        return self.controller_name if self.controller_name.startswith("/") else "/"+self.controller_name

    def start(self):
        super(AbstractControllerPluginServer, self).start()
        ut.register_client(self.ns, self.get_controller_name())
        rospy.on_shutdown(lambda: ut.unregister_client(self.ns, self.get_controller_name()))

    def __get_clean_ns(self):
        return self.ns[1:] if self.ns.startswith("/") else self.ns

    @property
    def query_service_name(self):
        return self.get_controller_name()+"/query" if self.controller_name != "" else ""

    @property
    def query_service_type(self):
        return ControllerQuery

    def generate_query_request(self, net_id, variable, meta_info):
        return ControllerQueryRequest(net_id, self.__get_clean_ns(), variable, meta_info)

    def query_controller(self, net_id, variable, meta_info={}):
        return ut.call_service(
            self.query_service_name,
            self.query_service_type,
            self.generate_query_request(net_id, variable, meta_info)
        )

    @property
    def update_service_name(self):
        return self.get_controller_name()+"/update" if self.controller_name != "" else ""

    @property
    def update_service_type(self):
        return ControllerUpdate

    def generate_update_request(self, net_id, variable, value, meta_info):
        a = self.__get_clean_ns()
        print self.ns, type(self.ns), a, type(a)
        return ControllerUpdateRequest(net_id, self.__get_clean_ns(), variable, value, meta_info)

    def update_controller(self, net_id, variable, value=None, meta_info={}):
        return ut.call_service(
            self.update_service_name,
            self.update_service_type,
            self.generate_update_request(net_id, variable, value, meta_info)
        )

