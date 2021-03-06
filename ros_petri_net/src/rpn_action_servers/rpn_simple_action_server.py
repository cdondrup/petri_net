# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import actionlib
import utils as ut
from ros_petri_net_msgs.srv import RPNQuery, RPNQueryRequest
from ros_petri_net_msgs.srv import RPNUpdate, RPNUpdateRequest


class RPNSimpleActionServer(actionlib.SimpleActionServer):
    (QUERY_ALL, QUERY_LOCAL, QUERY_REMOTE) = (RPNQueryRequest.ALL, RPNQueryRequest.LOCAL, RPNQueryRequest.REMOTE)
    (UPDATE_ALL, UPDATE_LOCAL, UPDATE_REMOTE) = (RPNUpdateRequest.ALL, RPNUpdateRequest.LOCAL, RPNUpdateRequest.REMOTE)

    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):
        self.ns = name
        actionlib.SimpleActionServer.__init__(self,
            name=name,
            ActionSpec=ActionSpec,
            execute_cb=execute_cb,
            auto_start=auto_start
        )

    def get_goal_id(self):
        return self.current_goal.get_goal_id().id.replace('/','').replace('-','_').replace('.','_')

    def query_kb(self, type, attr, meta_info=None):
        query_service = "/"+self.get_goal_id()+"/query"

        return ut.call_service(
            query_service,
            RPNQuery,
            RPNQueryRequest(
                type=type,
                attr=attr,
                meta_info=meta_info
            )
        )


    def update_kb(self, type, attr, value, meta_info=None):
        update_service = "/"+self.get_goal_id()+"/update"

        return ut.call_service(
            update_service,
            RPNUpdate,
            RPNUpdateRequest(
                type=type,
                attr=attr,
                value=value,
                meta_info=meta_info
            )
        )

