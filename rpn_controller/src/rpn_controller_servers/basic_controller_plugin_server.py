#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@author: Christian Dondrup
"""

import rospy
import actionlib
from rpn_recipe_planner_msgs.msg import RosServerAction
from rpn_controller_servers.abstract_controller_plugin_server import AbstractControllerPluginServer


class BasicControllerPluginServer(AbstractControllerPluginServer):
    def __init__(self, ns, ActionSpec=RosServerAction, goal_cb=None, cancel_cb=actionlib.nop_cb, auto_start=True):
        super(BasicControllerPluginServer, self).__init__(ns, ActionSpec, goal_cb, cancel_cb, auto_start)
        self.__cn = rospy.get_param("rpn_controller_name")
        print "GOT CONTROLLER NAME AS:", self.__cn

    @property
    def controller_name(self):
        return self.__cn


if __name__ == "__main__":
    rospy.init_node("example_controller_plugin_server")
    p = ExampleController(rospy.get_name())
    rospy.spin()
