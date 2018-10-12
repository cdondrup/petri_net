from pnp_kb.external_knowledge_base import ExternalKnowledgeBase
import utils as ut
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryRequest
from rpn_recipe_planner_msgs.srv import RPInform, RPInformRequest
from ontologenius.srv import OntologeniusService, OntologeniusServiceRequest
import json


class RPKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(RPKnowledgeBase, self).__init__(net_id)

    def query(self, variable, meta_info=None):
        a = getattr(self, variable)
        print "+++ QUERY +++", variable, a
        return a

    def update(self, variable, value, meta_info=None):
        raise AttributeError("Updates are not supported at the moment.")


