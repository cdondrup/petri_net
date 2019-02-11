#!/usr/bin/python
# -*- coding: utf-8 -*-

from pnp_gen.generator import Generator
from pnp_actions.pn_action import PNAction
from pnp_actions.recovery import Recovery, Before, During, After
from pnp_kb.queries import LocalQuery, RemoteQuery, Query
from pnp_kb.external_knowledge_base import ExternalKnowledgeBase
from pnp_gen.operations import BooleanAssertion, Comparison
from threading import Lock
from pprint import pprint


class MyExternalKnowledgeBase(ExternalKnowledgeBase):
    """ Very simple external knowledge base example which just saves
    data in a dict and returns it when queried."""

    def __init__(self, net_id):
        super(MyExternalKnowledgeBase, self).__init__(net_id)

    def query(self, variable, meta_info=None):
        if meta_info is not None:
            print "+++ Warning! Meta info field is ignored. +++"
        print "--- QUERY ---", variable, getattr(self, variable)
        return getattr(self, variable)

    def update(self, variable, value, meta_info=None):
        if meta_info is not None:
            print "+++ Warning! Meta info field is ignored. +++"
        setattr(self, variable, value)
        print "--- UPDATE ---", variable, getattr(self, variable)


class Example(object):
    def __init__(self):
        # Creating an instance of the generator
        gen = Generator()
        # Creating a new net with the name "test_net" using out External KB
        p, net = gen.create_net("test_net", MyExternalKnowledgeBase)


        pprint(net)


if __name__ == "__main__":
    Example()

