=================
PNP Common
=================

This package holds the most common elements to any Petri-Net and the net itself. Hence, most of the basic functionality of a Petri-Net is represented here.

.. py:module:: pnp_common.pn_common

.. class:: PNBaseObject(name)

	The base class for almost all classes in this package. It holds the name of the object, aunique ID :class:`uuid.uuid4` created at construction, and both the local and global knowledge bases. It also contains a few basic logging mthods.

	:param str name: The name of this Petri-Net object.

	.. method:: add_kb(kb, external_kb)

		Adds the given knowledge bases to the object.

		:param kb: The local knowldege base used by the action. Normally just uses the default implementation :class:`pnp_kb.knowledgebase.KnowledgeBase`.
		:type kb: :class:`pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base used by the action.
		:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`

	.. method:: loginfo(text)

		Prints the given text together with the name of this object to the terminal.

		:param str text: The text to log to the terminal.

	.. method:: logwarn(text)

		Prints the given text together with the name of this object to the terminal.

		:param str text: The text to log to the terminal.

	.. method:: logerr(text)

		Prints the given text together with the name of this object to the terminal.

		:param str text: The text to log to the terminal.

.. py:module:: pnp_common.execution_token

.. class:: ExecutionToken(name)

	Inherits from :class:`~pnp_common.pn_common.PNBaseObject`. *Not used at the moment*.

	:param str name: The name of this Petri-Net object.

.. py:module:: pnp_common.petri_net

.. class:: PetriNet(name, external_kb[, initial_knowledge = None])

	The Petri-Net. Inherits from :class:`~pnp_common.pn_common.PNBaseObject`. this class holds all the places and transitions and offers method to manipulate them.

	:param str name: The name of this Petri-Net object.
	:param external_kb: The global knowldege base used by the action.
	:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`
	:param dict(str,...) initial_knowledge: The starting knowledge to be given to the :class:`pnp_kb.knowledgebase.KnowledgeBase` upon creation.

	.. method:: get_current_places(marking)

		Get the places with a current marking of >0.

		:param list(double) marking: The current marking of the net held by the :class:`pnp_execution.executor.Execution`.
		:return: A 2D list of the names of places with a marking of >0 and the marking of said place.
		:rtype: list(list(str), list(double))

	.. method:: get_current_transitions(trans)

		Get the list of transitions that should fire based on the list of active transitions passed in.

		:param list(double) trans: The list of active transitions marked with a >0.
		:return: The list of active transitions.
		:rtype: list(:class:`~pnp_common.transition.Transition`)

	.. method:: is_goal(marking[, any=False])

		Checks if a goal place has been reached.

		:param list(double) marking: The current marking of the net.
		:param bool any: Set to ``True`` if only one of the active places has to be a goal for this to return ``True``.
		:return: ``True`` or ``False`` depending on ``any`` and if a goal has been reached.
		:rtype: bool

	.. method:: is_fail(marking[, any=False])

		Checks if a fail place has been reached.

		:param list(double) marking: The current marking of the net.
		:param bool any: Set to ``True`` if only one of the active places has to be a fail for this to return ``True``.
		:return: ``True`` or ``False`` depending on ``any`` and if a fail place has been reached.
		:rtype: bool

	.. method:: is_place(marking, name[, any=False])

		Checks if all place or one of the places (depending on ``any``) have the name specified using :meth:`check_all_places` and :meth:`check_any_places`. This method is mainly used by :meth:`is_goal` and :meth:`is_fail`.

		:param list(double) marking: The current marking of the net.
		:param str name: The name to check the places against.
		:param bool any: Set to ``True`` if only one of the active places has to be a goal for this to return ``True``.
		:return: ``True`` or ``False`` depending on ``any`` and if the place(s) have the specified name.
		:rtype: bool

	.. method:: check_all_places(marking, name)

		Checks if all places with a marking >0 have the specified name.

		:param list(double) marking: The current marking of the net.
		:return: ``True`` or ``False`` if all the places have the specified name.
		:rtype: bool

	.. method:: check_any_places(marking, name)

		Checks if any place with a marking >0 has the specified name.

		:param list(double) marking: The current marking of the net.
		:return: ``True`` or ``False`` if any of the places has the specified name.
		:rtype: bool

	.. method:: d_minus
		:property:

		The D- matrix of outgoing arcs.

		:raises AttributeError: When trying to set the value. This matrix is calculated based on transitions and places and cannot be set.

	.. method:: d_plus
		:property:

		The D+ matrix of incoming arcs.

		:raises AttributeError: When trying to set the value. This matrix is calculated based on transitions and places and cannot be set.

	.. method:: d
		:property:

		The D matrix of incoming arcs. Calculated as D = D+ - D-.

		:raises AttributeError: When trying to set the value. This matrix is calculated based on transitions and places and cannot be set.

	.. method:: add_transition(transition)

		Adds a transition to the internal list of transitions after adding the knowledgbases to it.

		:param transition: The transition to add.
		:type transition: :class:`~pnp_common.transition.Transition`

	.. method:: add_place(place)

		Adds a place to the internal list of places after adding the knowledgbases to it.

		:param place: The place to add.
		:type place: :class:`~pnp_common.place.Place`

	.. method:: add_init_place(place)

		Adds a place to the internal list and makes it the initial starting place of the net.

		:param place: The place to add.
		:type place: :class:`~pnp_common.place.Place`

	.. method:: transitions
		:property:

		:return: The list of transitions of the net.
		:rtype: :class:`numpy.array` (:class:`~pnp_common.transition.Transition`)

	.. method:: places
		:property:

		:return: The list of places of the net.
		:rtype: :class:`numpy.array` (:class:`~pnp_common.place.Place`)

	.. method:: num_places
		:property:

		:return: The number of places in the net.
		:rtype: int

	.. method:: num_transitions
		:property:

		:return: The number of transitions in the net.
		:rtype: int


.. py:module:: pnp_common.transition

.. class:: Arc(place[, weight=1])

	Inherits from :class:`~pnp_common.pn_common.PNBaseObject`. An arc that connects a place with a transition. This clas defines the place the arc connects to and the weight of the arc. The wieght detremines the marking that the :class:`~pnp_common.place.Place` needs in order to trigger the :class:`~pnp_common.transition.Transition` or the marking that the :class:`~pnp_common.place.Place` will have after the :class:`~pnp_common.transition.Transition` fired. This depends on if the arc is an incoming, i.e. from a place to a transition, or an outgoing, i.e. from a transition to a place, arc. This is determined when the arc is added to the :class:`~pnp_common.transition.Transition`.

	:param place: The place this arc connest to.
	:type place: :class:`~pnp_common.place.Place`
	:param int weight: The weight of the arc.

.. class:: Transition(name[, incoming_arcs=None[, outgoing_arcs=None[, condition=True, atomic_action=None[, query=None]]]])

	Inherits from :class:`~pnp_common.pn_common.PNBaseObject`. Transitions are the constructs of the net that are responsible for starting the :class:`pnp_actions.atomic_action.AtomicAction` associated with it. This association is handleded inside the :class:`pnp_actions.pn_action.PNAction` constructor. Transitions can have multiple places connected to them which is defined via the incoming and outgoing arcs. Transitions can also be conditional. Hence, in adition to requireing the right amount of execution tokens in the places connected with incoming arcs, are condition or query can be attached to the transition which has to be ``True`` in order for the transition to be firing.

	:param str name: The name of the transition. By convention this is the name of the :class:`pnp_actions.atomic_action.AtomicAction` .T.<function identifier>.
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

	.. method:: evaluate_condition

		This method evaluates the query and condition passed during initialisation.

		:return: The truth value of the query or condition.
		:rtype: bool

	.. method:: execute_atomic_action(event)

		Executes the associated action if any by calling :meth:`pnp_actions.atomic_action.AtomicAction.start`.

		:param event: The event set when the execution of the action has finished.
		:type event: :class:`threading.Event`


.. py:module:: pnp_common.place

.. class:: Place(name[, atomic_action=None])

	Inherits from :class:`~pnp_common.pn_common.PNBaseObject`. Representation of a place. Mainly responsible for connecting transitions and monitoring the execution of actions.

	:param str name: The name of the place. By convention the name is the name of the :class:`pnp_actions.atomic_action.AtomicAction` .P.<function identifier>.
	:param atomic_action: The action aossociated ith the transition if any. Most places will have no action associated but are simply structural for the net.
	:type atomic_action: :class:`pnp_actions.atomic_action.AtomicAction`

	.. method:: monitor_atomic_action

		Calls the :meth:`pnp_actions.atomic_action.AtomicAction.monitor` method.

		:return: The monitor thread or None if no action has been associated.
		:rtype: :class:`threading.Thread`
