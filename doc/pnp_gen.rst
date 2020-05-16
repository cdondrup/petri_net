=================
PNP Generation
=================

This package deals with the generation of Petri-Nets. It provides the generator which is a factory class for nets and several logical, e.g. and, or, not, and other operations such as comparisons. All these classes are used to turn plans and domains into actual Petri-Nets.

.. py:module:: pnp_gen.generator

.. class:: Generator

	The generator is a factory class to generate Petri-Nets. It offers various methods to create specific net structures such as loop, forks, and joins. These methods can be used by other classes to create a fully functioning net.

	.. method:: create_net(name, external_kb[, initial_knowledge=None])

		This method creates a new mepty :class:`pnp_common.petri_net.PetriNet`. This net only contains one :class:`pnp_common.place.Place` called *Init* which is the starting place of the net.

		:param str name: The name of this Petri-Net object.
		:param external_kb: The global knowldege base used by the action.
		:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`
		:param dict(str,...) initial_knowledge: The starting knowledge to be given to the :class:`pnp_kb.knowledgebase.KnowledgeBase` upon creation.
		:return: The end place of the net and the net itself.
		:rtype: tuple(:class:`pnp_common.place.Place`, :class:`pnp_common.petri_net.PetriNet`)

	.. method:: create_place(name=None, atomic_action=None)

		Create a :class:`pnp_common.place.Place` with the given name or a sequentially numbered :class:`pnp_common.place.Place` called P<num> where <num> is a running index.

		:param str name: The name of the new :class:`pnp_common.place.Place`. If omitted, the name will P<num>.
		:param atomic_action: The action associated with the place if any.
		:type atomic_action: :class:`pnp_actions.atomic_action.AtomicAction`
		:return: The create place.
		:rtype: :class:`pnp_common.place.Place`

	.. method:: create_transition(self[, name=None[, atomic_action=None[, condition=True[, query=None[, incoming_arcs=None[, outgoing_arcs=None]]]]]])

		Create a :class:`pnp_common.transition.Transition` with the given name or a sequentially numbered :class:`pnp_common.transition.Transition` called T<num> where <num> is a running index.

		:param str name: The name of the new :class:`pnp_common.transition.Transition`. If omitted, the name will T<num>.
		:param incoming_arcs: The arcs leading to the transition.
		:type incoming_arcs: list(:class:`Arc`)
		:param outgoing_arcs: The arcs going out from the transition.
		:type outgoing_arcs: list(:class:`Arc`)
		:param condition: A condition under which the transition is executed once the marking allows. this can either be simple boolean value or a callable method that returns a boolean.
		:type condition: bool or callable
		:param atomic_action: The action aossociated ith the transition if any. Most transitions will have no action associated but are simply structural for the net.
		:type atomic_action: :class:`pnp_actions.atomic_action.AtomicAction`
		:param query: A knowledge base query that returns a boolean. If a query is given, it has to be ``True`` for the transition to fire. A query superseeds a given condition.
		:type query: :class:`pnp_kb.queries.AbstractAtomicQuery`
		:return: The create transition.
		:rtype: :class:`pnp_common.transition.Transition`

	.. method:: add_action(net, current_place, action)

		Adds a :class:`pnp_actions.pn_action.PNAction` to the net connected to the ``current_place``.

		:param net: The Petri-Net to add the action to.
		:type net: :class:`pnp_commom.petri_net.PetriNet`
		:param current_place: The place to connect the :class:`pnp_actions.pn_action.PNAction` to.
		:type current_place: :class:`pnp_common.place.Place`
		:param action: The action to add to the net.
		:type action: :class:`pnp_actions.pn_action.PNAction`
		:return: The end place of the net and the net itself.
		:rtype: tuple(:class:`pnp_common.place.Place`, :class:`pnp_common.petri_net.PetriNet`) 

	.. method:: add_concurrent_action(net, current_place, actions)

		Adds several :class:`pnp_actions.pn_action.PNAction` as concurrently executed actions to the net connected to the ``current_place``.

		:param net: The Petri-Net to add the action to.
		:type net: :class:`pnp_commom.petri_net.PetriNet`
		:param current_place: The place to connect the :class:`pnp_actions.pn_action.PNAction` to.
		:type current_place: :class:`pnp_common.place.Place`
		:param actions: The list of concurrent actions to add to the net.
		:type actions: list(:class:`pnp_actions.pn_action.PNAction`)
		:return: The end place of the net and the net itself.
		:rtype: tuple(:class:`pnp_common.place.Place`, :class:`pnp_common.petri_net.PetriNet`) 

	.. method:: add_while_loop(net, current_place, actions, query)

		This meethod creates a while loop structure and adds it to the net. The loop executes the actions given in sequence. Loops will be sequentially numbered. The given query has to be true for the loop to be executed. Once the query becomes false, the loop terminates or will not be executed to begin with.

		:param net: The Petri-Net to add the action to.
		:type net: :class:`pnp_commom.petri_net.PetriNet`
		:param current_place: The place to connect the :class:`pnp_actions.pn_action.PNAction` to.
		:type current_place: :class:`pnp_common.place.Place`
		:param actions: The list of actions to be executed in sequence in the loop.
		:type actions: list(:class:`pnp_actions.pn_action.PNAction`)
		:param query: A knowledge base query that returns a boolean. If a query is given, it has to be ``True`` for the loop to execute.
		:type query: :class:`pnp_kb.queries.AbstractAtomicQuery`
		:return: The end place of the net and the net itself.
		:rtype: tuple(:class:`pnp_common.place.Place`, :class:`pnp_common.petri_net.PetriNet`)

	.. method:: create_fork(current_place, num)

		This method creates a fork. Forks have a single :class:`pnp_common.transition.Transition` and several instances of :class:`pnp_common.place.Place`. This single transition is connected to all the places via outgoing arcs.

		:param current_place: The place to connect the fork to.
		:type current_place: :class:`pnp_common.place.Place`
		:param int num: The number of instances of :class:`pnp_common.place.Place` to create and connect to the :class:`pnp_common.transition.Transition`.
		:return: The list of end places of the fork and the transition.
		:rtype: tuple(list(:class:`pnp_common.place.Place`), :class:`pnp_common.transition.Transition`) 

	.. method:: create_join

		This method creates a single :class:`pnp_common.transition.Transition` and a single :class:`pnp_common.place.Place`. The place is connected to the transition via an outgoing arc. The purpose of this construct is to have the incoming arcs of the transition connect to several places and join the execution into a single thread.

		:return: The place and the transition.
		:rtype: tuple(:class:`pnp_common.place.Place`, :class:`pnp_common.transition.Transition`) 

	.. method:: create_fail_place

		To gurantee correct naming, this method creates a place that will fail the net execution if reached. This name is checked in :meth:`pnp_common.petri_net.PetriNet.is_fail`.

		:return: A new place called ``Fail``.
		:rtype: :class:`pnp_common.place.Place`


	.. method:: add_goal(net, current_place)

		To gurantee correct naming, this method creates a place that will succeed the net execution if reached. This name is checked in :meth:`pnp_common.petri_net.PetriNet.is_goal`. Additionally, the new place is also diretcly added to the ``current_place`` given.

		:param net: The Petri-Net to add the place to.
		:type net: :class:`pnp_common.petri_net.PetriNet`
		:param current_place: The place to which to connect the new goal place.
		:type current_place: :class:`pnp_common.place.Place`
		:return: The goal place and the net itself.
		:rtype: tuple(:class:`pnp_common.place.Place`, :class:`pnp_common.petri_net.PetriNet`)

.. py:module:: pnp_gen.operations

.. class:: AbstractOperation

	This base class provides functionality to make all subclasses callable and provides a shared method to excute knowledge base queries. Inheriting classes will have to override the :meth:`run` method. 

	.. method:: run
		:abstractmethod:

		This method has to be overriden by inheriting classes. It should contain the actual logic of the operation and return a bool to indicate if the operation has been successful. 

		:rtype: bool

	.. method:: __call__(internal_kb, external_kb)

		The ``__call__`` method has been overriden to make the class executable. It saves the given knowledge bases as member variables and executes the :meth:`run` method.

		:param internal_kb: The local knowldege base. Normally just uses the default implementation :class:`pnp_kb.knowledgebase.KnowledgeBase`.
		:type internal_kb: :class:`pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base.
		:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`
		:return: The result of :meth:`run`
		:rtype: bool

	.. method:: execute_query(query)

		This is a convenience method. As most opertations need to execute some form of KB query, this method can be used to do so. Since often the query is not known to the actual operation, this method also provides functionality to handle simple operations that always succeed or fail. If that is the case the passed in query can also be a bool and is then just piped through. This method is often used in the :meth:`run` method.

		:param query: The query object to be executed as part of this operation.
		:type query: :class:`pnp_kb.queries.Query` or :class:`pnp_kb.queries.LocalQuery` or :class:`pnp_kb.queries.RemoteQuery` or bool
		:return: The result of the query if the query paramter was not of type bool. Otherwise, simply returns the bool itself.
		:rtype: bool


.. class:: Exists(query)

	Inherits form :class:`AbstractOperation`. This operation simply checks if a variable with a specific name exists in the KB.

	:param query: The query object to be executed as part of this operation.
	:type query: :class:`pnp_kb.queries.Query` or :class:`pnp_kb.queries.LocalQuery` or :class:`pnp_kb.queries.RemoteQuery` or bool

	.. method:: run

		:return: ``ture`` if the result of :meth:`AbstractOperation.execute_query` is not ``None``.
		:rtype: bool


.. class:: Operation(operator, queries)

	Inherits form :class:`AbstractOperation`. This class executes generic python operations using :mod:`operator`.

	:param str operator: The name of the operator in the :mod:`operator` module. Examples include: eq, nq, lt, gt, ...
	:param query: A list of query objects. Their results will be compared using the operator given.
	:type query: list(:class:`pnp_kb.queries.Query` or :class:`pnp_kb.queries.LocalQuery` or :class:`pnp_kb.queries.RemoteQuery` or bool)

	.. method:: run

		:return: The result of the operation executed on the result of the queries in the list.
		:rtype: bool

.. class:: Comparison(operator, queries)

	Inherits from :class:`Operation` and represents a class to compare two things. Works exactly the same as :class:`Operation` but makes sure the are only two queries given when created as all comparisons only support two arguments.

	:param str operator: The name of the comparison in the :mod:`operator` module. Examples include: eq, nq, lt, gt, ...
	:param query: A list of query objects. Their results will be compared using the operator given. Max size allowed is 2.
	:type query: list(:class:`pnp_kb.queries.Query` or :class:`pnp_kb.queries.LocalQuery` or :class:`pnp_kb.queries.RemoteQuery` or bool)
	:raises AttributeError: If more queries than two are given in the list.


.. class:: BooleanAssertion(operation, truth_value)

	Takes in an operation an asserts that the outcome is the same as the truth value provided. This is used for example in cases where you do not want an operation to be true. Most often, this is used internally to check the effects or preconditions of an action. These effects and preconditions are always tested for both the positive and negative outcome. One :class:`pnp_common.transition.Transition` checks the positive and one checks the negative. Which ever :class:`pnp_common.transition.Transition` has a ``true`` as a result becomes active and fires.

	:param operation: The operation you want to assert has the desired truth value.
	:type operation: :class:`AbstractOperation`
	:param bool truth_value: The truth value the has to be the result of the operation for the assertion to be correct.

	.. method:: invert

		This changes the ``truth_value`` to ``not(truth_value)``. Convenience method to create the assertion for the negative transition. As mentioned above the assertions are mainly used to check which :class:`pnp_common.transition.Transition` is supposed to fire in a net. Since assertions are binary, i.e. they are either true or false, there are always two transitions when ever a check occurs. One that checks the desired outcome and one that checks the undesired outcome. To generate the assertion for the undesired case, this method can be called after its creation.

	.. method:: __call__

		:return: ``true`` if the result of the operation is the same as the ``truth_value``. ``false`` otherwise.
		:rtype: bool

.. py:module:: pnp_gen.logical_operations

.. class:: AbstractLogicalOperation(operation1[, operation2[, operation3[, ...]]])

	This base class provides functionality to make all subclasses callable and provides a shared method to excute knowledge base queries. Inheriting classes will have to override the :meth:`run` method. It takes a number of operations as parameters that are supposed to be combined with the specific logical operation that implements this class.

	:param operation1: A operation to cobine with the other operations here using the logical operation.
	:type operation1: :class:`pnp_gen.operations.AbstractOperation` or :class:`AbstractLogicalOperation`
	:param operation2: A operation to cobine with the other operations here using the logical operation.
	:type operation2: :class:`pnp_gen.operations.AbstractOperation` or :class:`AbstractLogicalOperation`
	:param operation3: A operation to cobine with the other operations here using the logical operation.
	:type operation3: :class:`pnp_gen.operations.AbstractOperation` or :class:`AbstractLogicalOperation`

	.. method:: run
		:abstractmethod:

		This method has to be overriden by inheriting classes. It should return the combined outcome of all the operations given as parameters at initialization. 

		:rtype: bool

	.. method:: __call__(internal_kb, external_kb)

		The ``__call__`` method has been overriden to make the class executable. It saves the given knowledge bases as member variables and executes the :meth:`run` method.

		:param internal_kb: The local knowldege base. Normally just uses the default implementation :class:`pnp_kb.knowledgebase.KnowledgeBase`.
		:type internal_kb: :class:`pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base.
		:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`
		:return: The result of :meth:`run`
		:rtype: bool

.. class:: LogicalAnd

	Inherits form :class:`AbstractLogicalOperation`. This combines all the operations with a logical ``and``. 

	.. method:: run

		:return: ``true`` if all logical operations returned ``true``. ``false`` otherwise.
		:rtype: bool

.. class:: LogicalOr

	Inherits form :class:`AbstractLogicalOperation`. This combines all the operations with a logical ``or``. 

	.. method:: run

		:return: ``true`` if at least one logical operation returned ``true``. ``false`` otherwise.
		:rtype: bool

.. class:: LogicalNot

	Inherits form :class:`AbstractLogicalOperation`. Only allowed to have a single operation as an argument.

	:raises AttributeError: If more then one operation is given at initialisation

	.. method:: run

		:return: ``true`` if the outcome of the operation is ``false``. ``false`` otherwise.
		:rtype: bool