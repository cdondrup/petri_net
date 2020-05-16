=================
PNP Execution
=================

.. py:module:: pnp_execution.executor

.. class:: Executor

	This class is responsible for executing a given net. It can concurrently execute multiple nets. To identify a specific net, a net ID is used. This net ID is the name of the net and should be unique.

	.. method:: add_net(net, marking)

		This method adds a net to the execution list of the executor.

		:param net: The Petri-Net to add to the execution list.
		:type net: :class:`pnp_common.petri_net.PetriNet`
		:param list(double) marking: The initial marking of the net.
		:return: The name of the net.
		:rtype: str

	..  method:: execute_net(net_id)

		This method starts the execution of the net with the given ``net_id``.

		:param str net_id: The name of the Petri-Net to execute.
		:return: The name of the net.
		:rtype: str

	.. method:: wait_for_net(net_id[, timeout=None])

		This method waits for a net with the given name to finish execution.

		:param str net_id: The name of the net to wait for.
		:param double timeout: An optional timeout after which has run out to finish waiting even if the net is still executing.

	.. method:: execute_net_and_wait(net_id[, timeout=None])

		Combines :meth:`execute_net` and :meth:`wait_for_net`.

		:param str net_id: The name of the net to wait for.
		:param double timeout: An optional timeout after which has run out to finish waiting even if the net is still executing.
		:return: The final status of the net. This can either be ``"succeeded"``, ``"failed"``, or ``""``. The latter only happens if the timeout runs out before the net finishes.
		:rtype: str

	.. method:: get_status(net_id)

		:param str net_id: The name of the net to get the status for.
		:return: The status of the net. This can either be ``"succeeded"``, ``"failed"``, or ``""``. The latter only happens if the net has not finished, yet.
		:rtype: str

	.. method:: __run(net_id)

		This method executes the net with the given ID. It is used as the execution target for a :class:`threading.Thread` by the :meth:`execute_net` method and should never be called directly.

		:param str net_id: The name of the net to run.

	.. method:: check_num_tokens(marking, d_minus)

		This method checks which transitions should fire based on the wight of the arcs specified in :attr:`pnp_common.petri_net.PetriNet.d_minus` matrix.

		:param marking: The current marking of the net.
		:type marking: :class:`numpy.array` (double)
		:param d_minus: The D- matrix of the net.
		:type d_minus: :class:`numpy.array` (double)
		:return: A list of 0 and 1 that indicates which transitions should fire based on the marking.
		:rtype: :class:`numpy.array` (double)

	.. method:: check_conditions(trans, transitions)

		This method checks the conditions and querys of all the :class:`pnp_common.transition.Transition` that should fire. This uses the :meth:`pnp_common.transition.Transition.evaluate_condition` method.

		:param trans: The list of transitions that should fire based on the outcome of :meth:`check_num_tokens`.
		:type trans: :class:`numpy.array` (double)
		:param transitions: The list of transitions in the net.
		:type transitions: list(:class:`pnp_common.transition.Transition`)
		:return: A list of 0 and 1 that indicates which transitions should be fireing based on the condition.

	.. method:: execute_atomic_actions(trans, transitions, event)

		Executes the :class:`pnp_actions.atomic_action.AtomicAction` associated with all the transitions that should fire.

		:param trans: The list of transitions that should fire based on the outcome of :meth:`check_num_tokens`.
		:type trans: :class:`numpy.array` (double)
		:param transitions: The list of transitions in the net.
		:type transitions: list(:class:`pnp_common.transition.Transition`)
		:param event: The event that is triggered when any of the actions finishes.
		:type event: :class:`threading.Event`

	.. method:: monitor_atomic_actions(marking, places)

		Calls the :meth:`pnp_common.place.Place.monitor_atomic_action` method of all places that should be active based on the current marking of the net.

		:param marking: The current marking of the net.
		:type marking: :class:`numpy.array` (double)
		:param places: The places of the net.
		:type places: list(:class:`pnp_common.place.Place`)
		:return: A list of all monitor threads.
		:rtype: list(:class:`threading.Thread`)


