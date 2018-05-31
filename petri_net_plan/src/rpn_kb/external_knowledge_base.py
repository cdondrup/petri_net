from abc import ABCMeta, abstractmethod


class ExternalKnowledgeBase(object):
    __metaclass__ = ABCMeta

    def __init__(self, net_id):
        self.net_id = net_id

    @abstractmethod
    def query(self, query_string):
        return

    @abstractmethod
    def update(self, update_string):
        return

