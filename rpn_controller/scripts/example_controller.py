#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@author: Christian Dondrup
"""

import rospy
from rpn_controller.abstract_controller import AbstractController
from rpn_controller_msgs.srv import ControllerQuery, ControllerQueryResponse
from rpn_controller_msgs.srv import ControllerUpdate, ControllerUpdateResponse


class ExampleController(AbstractController):
    def __init__(self, name):
        super(ExampleController, self).__init__(name)

    def spin(self):
        print "The controller has started"
        while not rospy.is_shutdown():
            if len(self.servers.keys()) > 0:
                print "The available actions are:", self.servers.keys()
                server = raw_input("Start server> ")
                print self.servers[server]
        rospy.sleep(1.)

    def query_callback(self, req):
        print req
        return ControllerQueryResponse()

    def update_callback(self, req):
        print req
        return ControllerUpdateResponse()


if __name__ == "__main__":
    rospy.init_node("example_controller")
    p = ExampleController(rospy.get_name())
    p.spin()
