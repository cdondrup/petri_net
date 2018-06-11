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


class RPNSimpleActionServer(actionlib.SimpleActionServer):
    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):
        self.ns = name
        self.query_service = None
        self.update_service = None
        actionlib.SimpleActionServer.__init__(self,
            name=name,
            ActionSpec=ActionSpec,
            execute_cb=execute_cb,
            auto_start=auto_start
        )

    def query_kb(self, type, attr):
        if self.query_service is None:
            self.query_service = "/"+self.current_goal.get_goal_id().id.replace('/','').replace('-','_').replace('.','_')+"/query"

        return ut.call_service(
            self.query_service,
            PNQuery,
            PNQueryRequest(
                type=type,
                attr=attr
            )
        )


    def update_kb(self, type, attr, value):
        if self.update_service is None:
            self.update_service = "/"+self.current_goal.get_goal_id().id.replace('/','').replace('-','_').replace('.','_')+"/update"

        return ut.call_service(
            self.update_service,
            PNUpdate,
            PNUpdateRequest(
                type=type,
                attr=attr,
                value=value
            )
        )

