from pnp_kb.external_knowledge_base import ExternalKnowledgeBase
import utils as ut
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryRequest
from rpn_recipe_planner_msgs.srv import RPUpdate, RPUpdateRequest
import json


class RPKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(RPKnowledgeBase, self).__init__(net_id)

    def query(self, variable, meta_info=None):
        try:
            a = getattr(self, variable)
            print "+++ QUERY +++", variable, a
            return a
        except (AttributeError, TypeError):
            if meta_info is None:
                meta_info = {}
            else:
                try:
                    meta_info = json.loads(meta_info)
                except:
                    pass

            return self.__controller_query(variable, meta_info)

    def __controller_query(self, variable, meta_info={}):
        print "+++ CONTROLLER QUERY +++", variable, meta_info
        if not isinstance(variable, (str, unicode)):
            try:
                variable = json.dumps(variable)
            except (AttributeError, TypeError) as e:
                print e
        meta_info = meta_info if isinstance(meta_info, (str, unicode)) else json.dumps(meta_info)

        r = ut.call_service(
            "/"+self.net_id.replace('-','_')+"/query",
            RPQuery,
            RPQueryRequest(
                variable=variable,
                meta_info=meta_info
            )
        )
        print "+++ CONTROLLER REPLY:", r
        try:
            return json.loads(r.result)
        except ValueError:
            return r.result

    def update(self, variable, value, meta_info={}):
        if variable == "CONTROLLER":
            print "+++ CONTROLLER UPDATE +++", variable, value, meta_info
            value = value if isinstance(value, (str, unicode)) else json.dumps(value)
            meta_info = meta_info if isinstance(meta_info, (str, unicode)) else json.dumps(meta_info)
            print type(value), type(meta_info)
            r = ut.call_service(
                "/"+self.net_id.replace('-','_')+"/update",
                RPUpdate,
                RPUpdateRequest(
                    value=value,
                    meta_info=meta_info
                )
            )
        else:
            setattr(self, variable, value)
            print "+++ UPDATE +++", variable, value

