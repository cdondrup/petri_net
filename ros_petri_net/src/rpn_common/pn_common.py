import rospy
from abc import ABCMeta, abstractmethod


class PNBaseObject(object):
    def __init__(self, name):
        self.name = name

    def loginfo(self, text):
        rospy.loginfo(self.name + ': ' + text)

    def logwarn(self, text):
        rospy.logwarn(self.name + ': ' + text)

    def logerr(self, text):
        rospy.logerr(self.name + ': ' + text)

