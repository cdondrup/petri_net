import rospy
from abc import ABCMeta, abstractmethod
import uuid


class PNBaseObject(object):
    def __init__(self, name):
        self.name = name
        self.id = str(uuid.uuid4())
        self.kb = None
        self.external_kb = None

    def add_kb(self, kb, external_kb):
        self.kb = kb
        self.external_kb = external_kb

    def loginfo(self, text):
        rospy.loginfo(self.name + ': ' + text)

    def logwarn(self, text):
        rospy.logwarn(self.name + ': ' + text)

    def logerr(self, text):
        rospy.logerr(self.name + ': ' + text)

    def __str__(self):
        return name

