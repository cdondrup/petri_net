from pn_common import PNBaseObject


class Place(PNBaseObject):
    def __init__(self, name, atomic_action=None):
        super(Place, self).__init__(name)
        self.atomic_action = atomic_action

    def monitor_atomic_action(self):
        if self.atomic_action:
            self.loginfo("Monitoring atomic_action: '{}'".format(self.atomic_action))
            self.atomic_action.monitor(self.kb)
            # self.loginfo("Action '{}' finished".format(self.atomic_action))
        else:
            self.loginfo("No atomic_action")
        return

    def __str__(self):
        return self.name + ((", atomic_action: " + str(self.atomic_action)) if self.atomic_action else "")

    def __repr__(self):
        return self.__str__()
        # from pprint import pformat
        # return pformat(vars(self), indent=0, width=2)

