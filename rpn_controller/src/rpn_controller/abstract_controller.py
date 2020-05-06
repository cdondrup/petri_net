# -*- coding: utf-8 -*-
"""
@author: Christian Dondrup
"""

import rospy
import rostopic
import roslib
from abc import ABCMeta, abstractmethod
from actionlib import ActionClient
from rpn_controller_msgs.srv import ControllerQuery, ControllerQueryResponse
from rpn_controller_msgs.srv import ControllerUpdate, ControllerUpdateResponse
from rpn_controller_msgs.srv import ControllerRegisterAction, ControllerRegisterActionResponse
from rpn_controller_msgs.srv import ControllerUnregisterAction, ControllerUnregisterActionResponse


GT = "goal_type"
S  = "server"


class AbstractController(object):
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name
        self.loginfo(self.name, "Starting")
        self.__servers = {}
        rospy.Service("~register_server", ControllerRegisterAction, self.__register_callback)
        rospy.Service("~unregister_server", ControllerUnregisterAction, self.__unregister_callback)
        rospy.Service("~query", ControllerQuery, self.query_callback)
        rospy.Service("~update", ControllerUpdate, self.update_callback)
        self.loginfo(self.name, "started")

    def loginfo(self, name, text):
        rospy.loginfo("[{name}]: {message}".format(name=name, message=text))

    def get_registered_server_names(self):
        return self.__servers.keys()

    def get_num_registered_servers(self):
        return len(self.get_registered_server_names())

    def get_server_client(self, name):
        return self.__servers[name][S]

    def get_server_goal_type(self, name):
        return self.__servers[name][GT]

    def get_server_goal(self, name):
        return self.get_server_goal_type(name)()

    def __get_goal_type(self, action_name):
        topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
        # remove "Action" string from goal type
        assert("Action" in topic_type)
        return roslib.message.get_message_class(topic_type[:-10]+"Goal")

    def __get_action_type(self, action_name):
        topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
        # remove "Goal" string from action type
        assert("Goal" in topic_type)
        return roslib.message.get_message_class(topic_type[:-4])

    def __register_callback(self, req):
        name = req.action_name[1:] if req.action_name[0] == "/" else req.action_name
        rospy.loginfo("Registering '%s' action server" % name)
        if name not in self.__servers:
            self.__servers[name] = {
                S: ActionClient(name, self.__get_action_type(name)),
                GT: self.__get_goal_type(name)
            }
            rospy.loginfo("Waiting for '%s' action server to start." % name)
            self.__servers[name][S].wait_for_server()
            rospy.loginfo("'%s' action server started." % name)
            print self.__servers
            return ControllerRegisterActionResponse(True)
        else:
            rospy.logwarn("'%s' already registered. Won't do anything." % name)
            return ControllerRegisterActionResponse(False)

    def __unregister_callback(self, req):
        name = req.action_name[1:] if req.action_name[0] == "/" else req.action_name
        rospy.loginfo("Unregistering '%s' action server" % name)
        if name in self.__servers:
            del self.__servers[name]
            return ControllerUnregisterActionResponse(True)
        else:
            rospy.logwarn("'%s' not registered. Won't do anything." % name)
            return ControllerUnregisterActionResponse(False)

    @abstractmethod
    def query_callback(self, req):
        return

    @abstractmethod
    def update_callback(self, req):
        return

