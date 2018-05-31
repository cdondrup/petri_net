from abstract_knowledgebase import AbstractKnowledgeBase
from abc import ABCMeta, abstractmethod


class ExternalKnowledgeBase(AbstractKnowledgeBase):
    __metaclass__ = ABCMeta

    def __init__(self, net_id):
        self.net_id = net_id

