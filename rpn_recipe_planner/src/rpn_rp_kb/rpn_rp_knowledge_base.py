from pnp_kb.external_knowledge_base import ExternalKnowledgeBase
import utils as ut
from rpn_recipe_planner_msgs.srv import RPQuery, RPQueryRequest
from rpn_recipe_planner_msgs.srv import RPInform, RPInformRequest
from ontologenius.srv import OntologeniusService, OntologeniusServiceRequest
import json


class RPKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(RPKnowledgeBase, self).__init__(net_id)
        # self.shops = ["Costa", "Starbucks"]
        self.ontology_srv = "/ontologenius/"
        self.call_ontology("actions", "close", "")
        # self.ontology = {
            # "a coffee shop": self.shops,
            # "coffee shop": self.shops
        # }

    def call_ontology(self, type_, action, param):
        print "ONTO", type_, action, param
        return ut.call_service(
            self.ontology_srv+type_,
            OntologeniusService,
            OntologeniusServiceRequest(
                action=action,
                param=param
            )
        ).values

    def get_names(self, t, l):
        for i, e in enumerate(l):
            r = self.call_ontology(t, "getName", e)
            if r:
                # l[i] = r[0].title()
                l[i] = r[0].lower()
        return l

    def ontology_query(self, param, onto_type="individual", onto_action="find"):
        t_individual = "individual"
        t_class = "class"
        types = (t_individual, t_class)

        if onto_type not in types:
            raise ValueError("'{}' unknown ontology type query. Must be: ({})".format(type_, ', '.join(types)))
        if not isinstance(param, (str, unicode)):
            raise TypeError()

        param = param.lower()

        r = self.get_names(onto_type, self.call_ontology(onto_type, onto_action, param))
        if len(r) == 1:
            return r[0]
        return r

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

            try:
                return self.ontology_query(variable, **meta_info)
            except (AttributeError, TypeError):
                return self.user_query(variable, meta_info)

    def user_query(self, variable, meta_info=None):
        print "+++ USER QUERY +++", variable, type(variable), meta_info, type(meta_info)
        if not isinstance(variable, (str, unicode)):
            try:
                variable = json.dumps(variable)
            except (AttributeError, TypeError) as e:
                print e

        r = ut.call_service(
            "/"+self.net_id.replace('-','_')+"/query",
            RPQuery,
            RPQueryRequest(
                return_value=variable,
                **meta_info
            )
        )
        print r
        try:
            return json.loads(r.result)
        except ValueError:
            return r.result

    def update(self, variable, value, meta_info=None):
        if meta_info is None:
            meta_info = {}
        else:
            try:
                meta_info = json.loads(meta_info)
            except:
                pass

        if variable == "USER":
            print "+++ USER UPDATE +++", variable, value, meta_info
            r = ut.call_service(
                "/"+self.net_id.replace('-','_')+"/inform",
                RPInform,
                RPInformRequest(
                    return_value=value,
                    **meta_info
                )
            )
        else:
            raise AttributeError("Other updates apart from informing the user are not supported at the moment.")
        return None


