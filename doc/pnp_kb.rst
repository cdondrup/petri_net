====================
PNP Knowledge Bases
====================

.. py:module:: pnp_kb.abstract_knowledgebase

.. class:: AbstractKnowledgeBase

	This abstract implementation provides abstract methods for ``query`` and ``update`` to make sure that all implementing classes provide those methods. It does not provide any other functionality.

	.. method:: query(variable[, meta_info=None])

		Abstract method. Needs to contain functionality to query the knowledge base.

		:param str variable: The name of the variable that is queried.
		:param str meta_info: A json representation of a :class:`dict` that contains additional information to be passed to the query.
		:return: A str or json representation of the queried object.
		:rtype: str

	.. method:: update(variable, value[, meta_info=None])

		Abstract method. Needs to contain functionality to query the knowledge base.

		:param str variable: The name of the variable that is queried.
		:param value: The value to be safed using the variable name provided.
		:type value: any
		:param str meta_info: A json representation of a :class:`dict` that contains additional information to be passed to the query.
		:return: A str or json representation of the queried object.
		:rtype: str


.. py:module:: pnp_kb.external_knowledge_base

.. class:: ExternalKnowledgeBase(net_id)

	The abstract base class for all external knowledge bases. Implements :class:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`. Apart form a constructor, this class does not provide any additional functionality.

	:param str net_id: The unique identifier of the net the knowledge base is used for.

.. py:module:: pnp_kb.knowledgebase

.. class:: KnowledgeBase([keyword=argument])

	Thread safe implemantation of :class:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`. Simply implements :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.query` and :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.update` using :meth:`setattr` and :meth:`getattr`. Also overrides :meth:`setattr` and :meth:`getattr` to be thread safe by using :class:`threading.Lock`. Takes any number of keyword and argument pairs as parameters to provide initial knowledge at creation time. For each pair, :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.update` is called.

.. py:module:: pnp_kb.updates

.. class:: AbstractAtomicUpdate(attr, value[, meta_info=None])

	The base class for all the update objects that can be used together with the knowledge bases. The update objects are not used in the :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.update` but call the :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.update`. This class is callable and has a single abstract method called :meth:`_run` which is called when the class is called.

	:param str attr: The name the new information should be stored under.
	:param value: This can be either a variable name to query for, an Operation, or an AbstractAtomicQuery. See :meth:`_call_op` for an explanation.
	:type value: any or :class:`pnp_gen.operations.Operation` or :class:`AbstractAtomicQuery`
	:param str meta_info: A json representation of a :class:`dict` with additional information passed to the query. How this is handled is up to the actual implementation of inheriting classes.

	.. method:: _run(kb, external_kb)

		This method needs to be overridden by implementing classes. It should execute the desired query by calling the :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.query` method of the desired implementation of :class:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`.

		:param kb: The local knowldege base implementation. Normally just uses the default implementation :class:`~pnp_kb.knowledgebase.KnowledgeBase`.
		:type kb: :class:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base implementation.
		:type external_kb: :class:`~pnp_kb.external_knowledge_base.ExternalKnowledgeBase`
		:return: The result of :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.update`. Normally, ``None``.
		:rtype: str

	.. method:: _call_op(op, kb, external_kb)

		Convenience method that calls a given :class:`pnp_gen.operations.Operation` or :class:`AbstractAtomicQuery`. This allows to recursively run queries by resolving a query for the value first before running the update. An example of this can be found in the implementation of :class:`LocalUpdate` 's ``_run`` method.

		.. code-block:: python

		    def _run(self, kb, external_kb):
		        kb.update(self.attr, self._call_op(self.value, kb, external_kb), meta_info=self.meta_info)	

	    	Here the method is used to call any operation, be it a :class:`pnp_gen.operations.Operation` or :class:`AbstractAtomicQuery` to get the name of the variable we want to query for first and the run the query.

	    	If the ``op`` argument is not actually callable with the right parameters, it will be assumed that there is no operation to call and the original values is returned.

.. class:: LocalUpdate

	Inherits from :class:`AbstractAtomicUpdate`. Overrides the :meth:`AbstractAtomicUpdate._run` method to update the knowledge base passed in as ``kb``. Uses :meth:`AbstractAtomicUpdate._call_op` to resolve any possible operations or nested queries before updating the knowledge base.

.. class:: RemoteQuery

	Inherits from :class:`AbstractAtomicUpdate`. Overrides the :meth:`AbstractAtomicUpdate._run` method to update the knowledge base passed in as ``external_kb``. Uses :meth:`AbstractAtomicUpdate._call_op` to resolve any possible operations or nested queries before updating the knowledge base.

.. class:: Query

	Inherits from :class:`AbstractAtomicUpdate`. Overrides the :meth:`AbstractAtomicUpdate._run` method to update both the knowledge base passed in as ``kb`` first and the ``external_kb``. Uses :meth:`AbstractAtomicUpdate._call_op` to resolve any possible operations or nested queries before updating the knowledge bases.

.. py:module:: pnp_kb.queries

.. class:: AbstractAtomicQuery(attr[, meta_info=None])

	The base class for all the query objects that can be used together with the knowledge bases. The query objects are not used in the :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.query` but call the :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.query`. This class is callable and has a single abstract method called :meth:`_run` which is called when the class is called.

	:param attr: This can be either a variable name to query for, an Operation, or an AbstractAtomicQuery. See :meth:`_call_op` for an explanation.
	:type attr: str or :class:`pnp_gen.operations.Operation` or :class:`AbstractAtomicQuery`
	:param str meta_info: A json representation of a :class:`dict` with additional information passed to the query. How this is handled is up to the actual implementation of inheriting classes.

	.. method:: _run(kb, external_kb)

		This method needs to be overridden by implementing classes. It should execute the desired query by calling the :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.query` method of the desired implementation of :class:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`.

		:param kb: The local knowldege base implementation. Normally just uses the default implementation :class:`~pnp_kb.knowledgebase.KnowledgeBase`.
		:type kb: :class:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base implementation.
		:type external_kb: :class:`~pnp_kb.external_knowledge_base.ExternalKnowledgeBase`
		:return: The result of :meth:`~pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase.query`
		:rtype: str

	.. method:: _call_op(op, kb, external_kb)

		Convenience method that calls a given :class:`pnp_gen.operations.Operation` or :class:`AbstractAtomicQuery`. This allows to recursively run queries by resolving a query for the varibale name first before running the actual query. An example of this can be found in the implementation of :class:`LocalQuery` 's ``_run`` method.

		.. code-block:: python

		    def _run(self, kb, external_kb):
    			return kb.query(self._call_op(self.attr, kb, external_kb), self.meta_info)

	    	Here the method is used to call any operation, be it a :class:`pnp_gen.operations.Operation` or :class:`AbstractAtomicQuery` to get the name of the variable we want to query for first and the run the query.

	    	If the ``op`` argument is not actually callable with the right parameters, it will be assumed that there is no operation to call and nothing happens.


.. class:: LocalQuery

	Inherits from :class:`AbstractAtomicQuery`. Overrides the :meth:`AbstractAtomicQuery._run` method to query the knowledge base passed in as ``kb``. Uses :meth:`AbstractAtomicQuery._call_op` to resolve any possible operations or nested queries before querying the knowledge base.

.. class:: RemoteQuery

	Inherits from :class:`AbstractAtomicQuery`. Overrides the :meth:`AbstractAtomicQuery._run` method to query the knowledge base passed in as ``external_kb``. Uses :meth:`AbstractAtomicQuery._call_op` to resolve any possible operations or nested queries before querying the knowledge base.

.. class:: Query

	Inherits from :class:`AbstractAtomicQuery`. Overrides the :meth:`AbstractAtomicQuery._run` method to query the knowledge base passed in as ``kb`` first. If the first query does not return a result meaning that the required information was not found, the ``external_kb`` is called. Uses :meth:`AbstractAtomicQuery._call_op` to resolve any possible operations or nested queries before querying the knowledge base.
