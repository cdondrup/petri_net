# from abc import ABCMeta, abstractmethod
from uuid import uuid4


class PNAction(object):
    def __init__(self, atomic_action, starting_transition, end_place, places, transitions):
        self.id = str(uuid4())
        self.atomic_action = atomic_action
        self.starting_transition = starting_transition
        self.end_place = end_place
        self.places = places
        self.transitions = transitions


