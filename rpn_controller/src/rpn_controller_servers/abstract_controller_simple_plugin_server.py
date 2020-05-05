# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import actionlib
import utils as ut
from dialogue_msgs.srv import DAQuery, DAQueryRequest
from dialogue_msgs.srv import DAInform, DAInformRequest


class DASimplePluginServer(actionlib.SimpleActionServer):
    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):
        self.ns = name
        self.query_service = None
        self.inform_service = None
        actionlib.SimpleActionServer.__init__(self,
            name=name,
            ActionSpec=ActionSpec,
            execute_cb=execute_cb,
            auto_start=auto_start
        )

    def start(self):
        actionlib.SimpleActionServer.start(self)
        ut.register_client(self.ns)
        rospy.on_shutdown(lambda: ut.unregister_client(self.ns))
        self.query_service = "/dialogue_arbiter/query"
        self.inform_service = "/dialogue_arbiter/inform"

    def query_controller(self, status, return_value):
        if self.query_service is None:
            rospy.logerr("Trying to access query service without the arbiter running!")
            return

        return ut.call_service(
            self.query_service,
            DAQuery,
            DAQueryRequest(
                action=self.ns[1:],
                status=status,
                return_value=return_value
            )
        )

    def inform_controller(self, status, return_value):
        if self.inform_service is None:
            rospy.logerr("Trying to access inform service without the arbiter running!")
            return

        return ut.call_service(
            self.inform_service,
            DAInform,
            DAInformRequest(
                action=self.ns[1:],
                status=status,
                return_value=return_value
            )
        )

