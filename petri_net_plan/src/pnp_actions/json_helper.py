import json
import importlib


class BaseObject(object):
    def __json__(self):
        # for k in self.__dict__.keys():
            # if isinstance(self.__dict__[k]):
                # pass
            # self.__dict__[k] = self.__dict__[k].__json__()
        return '{{"module": "{}", "cls": "{}", "data": {}}}'.format(self.__module__, self.__class__.__name__, json.dumps(self.__dict__))



class Decoder(json.JSONDecoder):
    def decode(self, json_object):
        o = json.loads(json_object)
        try:
            r = getattr(importlib.import_module(o["module"]), o["cls"])(**o["data"])
            r.__dict__ = o["data"]
            return r
        except KeyError:
            return json_object
