#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@author: Christian Dondrup
"""

import rospy
from rpn_controller.abstract_controller import AbstractController
from rpn_controller_msgs.srv import ControllerQuery, ControllerQueryResponse
from rpn_controller_msgs.srv import ControllerUpdate, ControllerUpdateResponse
from uuid import uuid4


class ExampleController(AbstractController):
    def __init__(self, name):
        super(ExampleController, self).__init__(name)

    def spin(self):
        print "The controller has started"
        while not rospy.is_shutdown():
            if self.get_num_registered_servers() > 0:
                print "The available actions are:", self.get_registered_server_names()
                server = raw_input("Start server> ")
                self.get_server_client(server).wait_for_server()
                goal = self.get_server_goal(server)
                goal.id = str(uuid4())
                self.get_server_client(server).send_goal(goal)
            rospy.sleep(3.)

    def query_callback(self, req):
        print "QUERY", req
        return ControllerQueryResponse()

    def update_callback(self, req):
        print "UPDATE", req
        return ControllerUpdateResponse()


if __name__ == "__main__":
    rospy.init_node("example_controller")
    p = ExampleController(rospy.get_name())
    p.spin()
