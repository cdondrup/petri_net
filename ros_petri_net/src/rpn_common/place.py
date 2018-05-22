from pn_common import PNBaseObject


class Place(PNBaseObject):
    def __init__(self, name, action=None):
        super(Place, self).__init__(name)
        self.action = action

    def monitor_action(self):
        if self.action:
            self.loginfo("Monitoring action: '{}'".format(self.action))
        else:
            self.loginfo("No action")
        return

