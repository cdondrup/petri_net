# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import actionlib
import utils as ut
from petri_net_msgs.srv import PNQuery, PNQueryRequest
from petri_net_msgs.srv import PNUpdate, PNUpdateRequest


class RPNActionServer(actionlib.ActionServer):
    (ALL, LOCAL, REMOTE) = (PNQueryRequest.ALL, PNQueryRequest.LOCAL, PNQueryRequest.REMOTE)

    def __init__(self, ns, ActionSpec, goal_cb, cancel_cb=actionlib.nop_cb, auto_start=True):
        self.ns = ns
        self.query_service = None
        self.update_service = None
        actionlib.ActionServer.__init__(self,
            ns=ns,
            ActionSpec=ActionSpec,
            goal_cb=goal_cb,
            cancel_cb=cancel_cb,
            auto_start=auto_start
        )

    def query_kb(self, gh, type, attr):
        if self.query_service is None:
            self.query_service = "/"+gh.get_goal_id().id.replace('/','').replace('-','_').replace('.','_')+"/query"

        return ut.call_service(
            self.query_service,
            PNQuery,
            PNQueryRequest(
                type=type,
                attr=attr
            )
        )


    def update_kb(self, gh, type, attr, value):
        if self.update_service is None:
            self.update_service = "/"+gh.get_goal_id().id.replace('/','').replace('-','_').replace('.','_')+"/update"

        return ut.call_service(
            self.update_service,
            PNUpdate,
            PNUpdateRequest(
                type=type,
                attr=attr,
                value=value
            )
        )

