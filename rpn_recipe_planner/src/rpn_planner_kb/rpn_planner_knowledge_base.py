from pnp_kb.external_knowledge_base import ExternalKnowledgeBase


class RPKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(RPKnowledgeBase, self).__init__(net_id)

    def query(self, variable, meta_info=None):
        a = getattr(self, variable)
        print "+++ QUERY +++", variable, a
        return a

    def update(self, variable, value, meta_info=None):
        print "+++ UPDATE +++", variable, value
        setattr(self, variable, value)


