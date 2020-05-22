.. Petri Net Machines documentation master file, created by
   sphinx-quickstart on Thu Apr 16 14:09:48 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Petri Net Machines
==============================================

Petri-Net Machines (PNM) for the Robot Operating System (ROS) are an alternative to statemachines. They have been designed to allow seemes and easy to use execution of sequences of (concurrent) actions. These actions are mainly ROS action servers which can simply be hooked into the system. This page introduces the functionality of PNMs and provides documentation. More info can be found here: https://arxiv.org/pdf/1909.06174.

Documentation
===============

.. toctree::
   :maxdepth: 2
   :caption: Tutorials:

   usage

.. toctree::
   :maxdepth: 2
   :caption: Petri-Net Plans API:

   pnp_actions
   pnp_common
   pnp_kb
   pnp_gen
   pnp_execution

.. toctree::
   :maxdepth: 2
   :caption: ROS Petri-Net API:

   rpn_action_servers

.. toctree::
   :maxdepth: 2
   :caption: RPN Recipe Planner API:

   rpn_controller_kb



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
