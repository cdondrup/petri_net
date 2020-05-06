# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy


def call_service(srv_name, srv_type, req):
    while not rospy.is_shutdown():
        try:
            s = rospy.ServiceProxy(
                srv_name,
                srv_type
            )
            s.wait_for_service(timeout=1.)
        except rospy.ROSException:
            rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
            rospy.sleep(1.)
        else:
            return s(req)


def loginfo(name, msg):
    rospy.loginfo("["+name+"]: "+msg)


def logwarn(name, msg):
    rospy.logwarn("["+name+"]: "+msg)


def logerr(name, msg):
    rospy.logerr("["+name+"]: "+msg)
