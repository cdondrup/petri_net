from pnp_kb.external_knowledge_base import ExternalKnowledgeBase
import utils as ut
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryRequest
import json


class RPKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(RPKnowledgeBase, self).__init__(net_id)
        self.shops = ["Costa", "Starbucks"]
        self.ontology = {
            "a coffee shop": self.shops,
            "coffee shop": self.shops
        }

    def query(self, variable, meta_info=None):
        try:
            a = getattr(self, variable)
            print "+++ QUERY +++", variable, a
            return a
        except (AttributeError, TypeError):
            print "+++ USER QUERY +++", variable, type(variable), meta_info, type(meta_info)
            if meta_info is not None:
                try:
                    meta_info = json.loads(meta_info)
                except:
                    pass

                if not isinstance(variable, (str, unicode)):
                    try:
                        variable = json.dumps(variable)
                    except (AttributeError, TypeError) as e:
                        print e

                r = ut.call_service(
                    "/"+self.net_id.replace('-','_')+"/query",
                    RPQuery,
                    RPQueryRequest(
                        status=meta_info["status"],
                        return_value=variable
                    )
                )
                print r
                try:
                    return json.loads(r.result)
                except ValueError:
                    return r.result

    def update(self, variable, value, meta_info=None):
        print "+++ UPDATE +++", variable, value
        return None


