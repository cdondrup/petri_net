from pn_common import PNBaseObject
import numpy as np


class PetriNet(PNBaseObject):
    def __init__(self, name):
        super(PetriNet, self).__init__(name)
        self._transitions = []
        self._places = []
        self._d_minus = None
        self._d_plus = None
        self._d = None

    def get_current_places(self, marking):
        res = [[], []]
        for p, m in zip(self.places[marking >= 1], marking[marking >= 1]):
            res[0].append(p.name)
            res[1].append(m)
        return res

    def is_finished(self, marking):
        for p in self.get_current_places(marking)[0]:
            if p != "Goal":
                return False
        return True

    @property
    def d_minus(self):
        if self._d_minus is None:
            self._d_minus = np.array([])
            for t in self._transitions:
                m = np.zeros(len(self._places), dtype=int)
                for a in t.incoming_arcs:
                    m[self._places.index(a.place)] = a.weight
                self._d_minus = np.append(self._d_minus, m).reshape(-1, m.shape[0])

        return self._d_minus

    @d_minus.setter
    def d_minus(self, value):
        raise AttributeError("This varible is calculated based on the places and transitions in the "\
                        "net. It cannot be set.")

    @property
    def d_plus(self):
        if self._d_plus is None:
            self._d_plus = np.array([])
            for t in self._transitions:
                m = np.zeros(len(self._places), dtype=int)
                for a in t.outgoing_arcs:
                    m[self._places.index(a.place)] = a.weight
                self._d_plus = np.append(self._d_plus, m).reshape(-1, m.shape[0])

        return self._d_plus

    @d_plus.setter
    def d_plus(self, value):
        raise AttributeError("This varible is calculated based on the places and transitions in the "\
                        "net. It cannot be set.")

    @property
    def d(self):
        if self._d is None:
            self._d = self.d_plus - self.d_minus

        return self._d

    @d.setter
    def d(self, value):
        raise AttributeError("This varible is calculated based on the places and transitions in the "\
                        "net. It cannot be set.")

    def add_transition(self, transition):
        self._transitions.append(transition)

    def add_place(self, place):
        self._places.append(place)

    @property
    def transitions(self):
        return np.array(self._transitions)

    @property
    def places(self):
        return np.array(self._places)

    @property
    def num_places(self):
        return len(self._places)

    @property
    def num_transitions(self):
        return len(self._transitions)

    def __repr__(self):
        from pprint import pformat
        return pformat(vars(self), indent=0, width=2)

