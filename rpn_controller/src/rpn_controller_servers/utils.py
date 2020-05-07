# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import rosservice
import socket
from rpn_controller_msgs.srv import ControllerRegisterAction, ControllerUnregisterAction
from threading import Thread
from Queue import Queue


def custom_rosservice_find(service_type):
    """
    Lookup services by service_type. Exteing functionality of rosservice.rosservice_find
    @param service_type: type of service to find
    @type  service_type: str
    @return: list of service names that use service_type
    @rtype: [str]
    """
    master = rosservice._get_master()
    matches = []
    try:
        _, _, services = master.getSystemState()
        for s, l in services:
            try:
                t = rosservice.get_service_type(s)
            except rosservice.ROSServiceIOException as e:
                rospy.logdebug(e)
            else:
                if t == service_type:
                    matches.append(s)
    except socket.error:
        raise rosservice.ROSServiceIOException("Unable to communicate with master!")
    print "MATCHES", matches
    return matches


def find_service_by_type(service_type):
    while True:
        try:
            return custom_rosservice_find(service_type)[0]
        except IndexError:
            rospy.logwarn("No service with type '%s' found. Retrying in 1 second. Enable debug mode for more information." % service_type)
            if rospy.core.is_shutdown_requested():
                return
            rospy.sleep(1.)


def unregister_client(name, controller_name=""):
    if controller_name != "":
        s = rospy.ServiceProxy(controller_name+"/unregister_server", ControllerUnregisterAction)
    else:
        s = rospy.ServiceProxy(find_service_by_type(ControllerUnregisterAction._type), ControllerUnregisterAction)
    try:
        s.wait_for_service(timeout=1.)
        s(action_name=name)
    except rospy.ROSException:
        rospy.logwarn("Unregistering unsuccessful. '%s' might still be registered with server." % name)


def register_client(name, controller_name=""):
    print "CONTROLLER NAME", controller_name
    if controller_name != "":
        s = rospy.ServiceProxy(controller_name+"/register_server", ControllerRegisterAction)
    else:
        s = rospy.ServiceProxy(find_service_by_type(ControllerRegisterAction._type), ControllerRegisterAction)
    print "CONTROLLER NAME", s
    try:
        s.wait_for_service(timeout=1.)
        s(action_name=name)
    except rospy.ROSException:
        rospy.logwarn("Something went horribly wrong when trying to register the '%s' client. Did the server die?" % name)


def call_service(srv_name, srv_type, req, blocking=True):

    def call(srv_name, srv_type, req, queue):
        while not rospy.is_shutdown():
            try:
                if srv_name == "":
                    s = rospy.ServiceProxy(find_service_by_type(srv_type._type), srv_type)
                else:
                    s = rospy.ServiceProxy(
                        srv_name,
                        srv_type
                    )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
                rospy.sleep(1.)
            else:
                queue.put(s(req))
                return

    queue = Queue()
    t = Thread(target=call, args=(srv_name, srv_type, req, queue))
    t.start()
    if blocking:
        t.join()
        return queue.get()
    return queue
