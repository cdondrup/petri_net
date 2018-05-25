from pn_common import PNBaseObject


class Place(PNBaseObject):
    def __init__(self, name, action=None):
        super(Place, self).__init__(name)
        self.action = action

    def monitor_action(self):
        if self.action:
            self.loginfo("Monitoring action: '{}'".format(self.action))
            self.action.monitor(self.kb)
            # self.loginfo("Action '{}' finished".format(self.action))
        else:
            self.loginfo("No action")
        return

    def __str__(self):
        return self.name + ((", action: " + str(self.action)) if self.action else "")

    def __repr__(self):
        return self.__str__()
        # from pprint import pformat
        # return pformat(vars(self), indent=0, width=2)

