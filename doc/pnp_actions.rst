=================
PNP Actions
=================

This packages efines the most basic action types. It includes an abstract atomic action that is the base class for all actions that can be executed by the petri net. It also defines its wrapper class which is the Petri Net action (PNAction). This class holds a atomic action and binds it to the petri net. It takes care of its execution and of handling the return values.

Additionally, this package contains recovery base classes and the knowledge base action that changes values in either of the knowledge bases.


.. py:module:: pnp_actions.atomic_action

.. class:: AtomicAction(name[, params=None])

	:param str name: The name of the action. Used to identify action servers to call for example.

	:param params: Possible parameters to be handed to the action. Not used in this abstract class but can be used by inheriting classes.
	:type params: dict or None

	This class serves as the base class for all actions to be executed. These actions have to be single actions and therefore should be atomic. This can for example be a :class:`~pnp_actions.kb_action.KBAction` or a ROS action server.

	.. attribute:: OUTCOMES
		:type: tuple(str)
		:value: ("succeeded", "preempted", "failed")

		Possible outcomes of atomic actions.

	.. attribute:: SUCCEEDED
		:type: str
		:value: "succeeded"

		Shortcut for :attr:`OUTCOMES` [0]

	.. attribute:: PREEMPTED
		:type: str
		:value: "preempted"

		Shortcut for :attr:`OUTCOMES` [1]

	.. attribute:: FAILED
		:type: str
		:value: "failed"

		Shortcut for :attr:`OUTCOMES` [2]


	.. method:: start(kb, external_kb, event)

		:param kb: The local knowldege base used by the action. Normally just uses the default implementation :class:`pnp_kb.knowledgebase.KnowledgeBase`.
		:type kb: :class:`pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base used by the action.
		:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`
		:param event: The event being triggered when the action has finished.
		:type event: :py:class:`threading.Event`

		This method is called when the atomic action is executed by the :meth:`pnp_common.transition.Transition.execute_atomic_action`. The knowledge bases and the event are populated and the execution thread is started. This thread calls the :meth:`run` method.

	.. method:: run(kb, external_kb)
		:abstractmethod: 
		
		:param kb: The local knowldege base used by the action. Normally just uses the default implementation :class:`pnp_kb.knowledgebase.KnowledgeBase`.
		:type kb: :class:`pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base used by the action.
		:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`

		This method has to be overriden by impelementing classes and should contain the action to execute. This method is called by the :meth:`start` method and should not be called directly.

	.. method:: trigger_event

		This method waits for the execution thread started in the :meth:`start` method to finish and then triggers the :py:class:`threading.Event` parameter given to the :meth:`start` method to notify the executing main loop that the action has finished. This method is called by the :meth:`monitor` method and should not be called directly.

	.. method:: monitor

		:return: The monitor thread
		:rtype: :py:class:`threading.Thread`

		This method starts a monitor thread using the :meth:`trigger_event` method. This method is called by the :meth:`pnp_common.place.Place.monitor_atomic_action` method.

	.. method:: succeeded
		:property:
		:abstractmethod:

		:return: ``True`` if successful and ``False`` if not.
		:rtype: boolean

		Needs to be overridden in implementing classes.

	.. method:: preempted
		:property:
		:abstractmethod:

		:return: ``True`` if preempted and ``False`` if not.
		:rtype: boolean

		Needs to be overridden in implementing classes.

	.. method:: failed
		:property:
		:abstractmethod:

		:return: ``True`` if failed and ``False`` if not.
		:rtype: boolean

		Needs to be overridden in implementing classes.


.. py:module:: pnp_actions.kb_action

.. class:: KBAction(name[, params=None])

	:param str name: The name of the action. Used to identify action servers to call for example.

	:param params: Parameters to be handed to the action. This needs to contain an ``"operation"`` key with an operation as a value.
	:type params: dict(str, :class:`pnp_kb.updates.AbstractAtomicUpdate`) or dict(str, :class:`pnp_kb.queries.AbstractAtomicQuery`)

	This class extends :class:`~pnp_actions.atomic_action.AtomicAction` and uses the same constructor. It is mean to perform updates and queries on the local or external knowledge base directly without needing to implement a specific atomic action for each of them. It, therefore, has close links to :obj:`pnp_kb`.

	.. method:: run(kb, external_kb)
		
		:param kb: The local knowldege base used by the action. Normally just uses the default implementation :class:`pnp_kb.knowledgebase.KnowledgeBase`.
		:type kb: :class:`pnp_kb.abstract_knowledgebase.AbstractKnowledgeBase`
		:param external_kb: The global knowldege base used by the action.
		:type external_kb: :class:`pnp_kb.external_knowledge_base.ExternalKnowledgeBase`

		Overrides the :meth:`~pnp_actions.atomic_action.AtomicAction.run` method. Thread safe execution of the operation passed in as :obj:`KBAction.params`.

	.. method:: get_state

		:return: ``True`` or ``False``
		:rtype: boolean

		Thread safe retrieval of the final state of the action. Retruns ``True`` if the :meth:`run` method has finished or ``False`` otherwise.

	.. method:: succeeded
		:property:
		:abstractmethod:

		:return: ``True`` or ``False``
		:rtype: boolean

		Uses :meth:`get_state` to check if the :meth:`run` method has finished and then returns ``True`` or ``False`` if it has not.

	.. method:: preempted
		:property:
		:abstractmethod:

		:return: ``False``
		:rtype: boolean

		Always returns ``False``. Action cannot be preempted.

	.. method:: failed
		:property:
		:abstractmethod:

		:return: ``False``
		:rtype: boolean

		Always returns ``False``. Action cannot fail.

.. py:module:: pnp_actions.pn_action

.. class:: PNAction(atomic_action[, recovery=None])

	This class holds and creates the Petri-Net structure of an :class:`~pnp_actions.atomic_action.AtomicAction` and an associated optional :class:`~pnp_actions.recovery.Recovery`. It creates all the necessary transitions, arcs and places. All :class:`PNAction` s will be stitched together based on the plan to create the final Petri-Net. This class is used by the :class:`pnp_gen.generator.Generator` to create the final net by connecting all the arcs and places.

	.. method:: set_up(net)

		This method triggers the generation of all the required components to connect this action to a given :class:`pnp_common.petri_net.PetriNet`. At the moment it only calls :meth:`apply_recovery_behaviours` which creates all these connections for the action to integrate it into the final Petri-Net.

		:param net: The Petri-Net to integrate this action into.
		:type net: :class:`pnp_common.petri_net.PetriNet`

	.. method:: apply_recovery_behaviours(net)

		This method connects the :class:`~pnp_actions.atomic_action.AtomicAction`, :class:`~pnp_actions.recovery.Recovery`, and :class:`pnp_common.petri_net.PetriNet`. While it does not determine where in the net the action goes precisely (this is done by the :class:`pnp_gen.generator.Generator`), it connects the recovery behaviours with the net. This is used for example for the :attr:`~pnp_actions.recovery.Recovery.RESTART_PLAN` and :attr:`~pnp_actions.recovery.Recovery.FAIL` recoveries.

		:param net: The Petri-Net to integrate this action into.
		:type net: :class:`pnp_common.petri_net.PetriNet`

	.. method:: add_places(places)

		Adds a list places to the internal list of places.

		:param places: The place(s) to add.
		:type places: :class:`pnp_common.place.Place` or list(:class:`pnp_common.place.Place`)

	.. method:: add_transitions(transitions)

		Adds a list transitions to the internal list of places.

		:param transitions: The transition(s) to add.
		:type transitions: :class:`pnp_common.transition.Transition` or list(:class:`pnp_common.transition.Transition`)

.. py:module:: pnp_actions.recovery

.. class:: During([preempted=None[, failed=None]])

	Inherits from :class:`dict`. This class holds lists of recovery behaviours that are supposed to be executed if something happens during the life time of a :class:`~pnp_actions.atomic_action.AtomicAction`. The keys for those lists are the :attr:`~pnp_actions.atomic_action.AtomicAction.OUTCOMES`. If any of these keys has a non-empty list as a value, the recovery actions in theses lists are executed if the outcome comes to pass. The :attr:`~pnp_actions.atomic_action.AtomicAction.SUCCEEDED` outcome always has an empty list as a value which means that the next place in the Petri-Net will be reached as planned.

	:param preempted: This is the list of actions to be performed if the :class:`~pnp_actions.atomic_action.AtomicAction` is preempted. The :class:`str` arguments can be :attr:`Recovery.RESTART_ACTION`, :attr:`Recovery.RESTART_PLAN`, :attr:`Recovery.SKIP_ACTION`, or :attr:`Recovery.FAIL`.
	:type preempted: list(str) or list(:class:`~pnp_actions.pn_action.PNAction`)
	:param failed: This is the list of actions to be performed if the :class:`~pnp_actions.atomic_action.AtomicAction` has failed. The :class:`str` arguments can be :attr:`Recovery.RESTART_ACTION`, :attr:`Recovery.RESTART_PLAN`, :attr:`Recovery.SKIP_ACTION`, or :attr:`Recovery.FAIL`.
	:type failed: list(str) or list(:class:`~pnp_actions.pn_action.PNAction`)

.. class:: Before([assertion=None[, recovery=None]])

	This class holds lists of recovery behaviours that are supposed to be executed if the assertion is ``True``. The assertion is checked **before** the action is executed.

	:param assertion: This is an assertion that is checked to determine if the recovery behaviour(s) should be executed.
	:type assertion: :class:`pnp_gen.operations.BooleanAssertion`
	:param failed: This is the list of actions to be performed if the assertion is true. The :class:`str` arguments can be :attr:`Recovery.RESTART_ACTION`, :attr:`Recovery.RESTART_PLAN`, :attr:`Recovery.SKIP_ACTION`, or :attr:`Recovery.FAIL`.
	:type failed: list(str) or list(:class:`~pnp_actions.pn_action.PNAction`)

.. class:: After([assertion=None[, recovery=None]])

	This class holds lists of recovery behaviours that are supposed to be executed if the assertion is ``True``. The assertion is checked **after** the action is executed.

	:param assertion: This is an assertion that is checked to determine if the recovery behaviour(s) should be executed.
	:type assertion: :class:`pnp_gen.operations.BooleanAssertion`
	:param failed: This is the list of actions to be performed if the assertion is true. The :class:`str` arguments can be :attr:`Recovery.RESTART_ACTION`, :attr:`Recovery.RESTART_PLAN`, :attr:`Recovery.SKIP_ACTION`, or :attr:`Recovery.FAIL`.
	:type failed: list(str) or list(:class:`~pnp_actions.pn_action.PNAction`)

.. class:: Recovery([before=None[, during=None[, after=None]]])

	The recovery object that is associated with an action. This holds all the before, during, and after recoveries that apply to a specific action. This recovery object is then handed to the :class:`~pnp_actions.pn_action.PNAction` constructor together with the :class:`~pnp_actions.atomic_action.AtomicAction` object to be integrated into the Petri-Net.


	:param before: The recovery behaviour(s) to be checked and executed **before** the action is executed.
	:type before: :class:`Before` or list(:class:`Before`)
	:param during: The recovery behaviour(s) to be checked and executed **during** the action is executed.
	:type during: :class:`During` or list(:class:`During`)
	:param after: The recovery behaviour(s) to be checked and executed **after** the action is executed.
	:type after: :class:`After` or list(:class:`After`)

	.. attribute:: RESTART_ACTION
		:type: str
		:value: "restart_action"

		Command to restart the action as a recovery behaviour.

	.. attribute:: RESTART_PLAN
		:type: str
		:value: "restart_plan"

		Command to restart the whole Petri-Net as a recovery behaviour.

	.. attribute:: SKIP_ACTION
		:type: str
		:value: "skip_action"

		Command to skip the action as a recovery behaviour.

	.. attribute:: FAIL
		:type: str
		:value: "fail"

		Command to fail the entire Petri-Net as a recovery behaviour.

