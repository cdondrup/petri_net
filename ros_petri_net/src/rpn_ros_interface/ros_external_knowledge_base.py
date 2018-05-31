from pnp_kb.external_knowledge_base import ExternalKnowledgeBase


class ROSExternalKnowledgeBase(ExternalKnowledgeBase):
    def __init__(self, net_id):
        super(ROSExternalKnowledgeBase, self).__init__(net_id)

    def query(self, variable):
        print "+++ EXTERNAL QUERY +++", variable
        return None

    def update(self, variable, value):
        print "+++ EXTERNAL UPDATE +++", variable, value
        return None

