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
        print "INFO: {}: {}".format(self.name, text)

    def logwarn(self, text):
        print "WARNING: {}: {}".format(self.name, text)

    def logerr(self, text):
        print "ERROR: {}: {}".format(self.name, text)

    def __str__(self):
        return self.name

