dobot_moveit
=============

Overview
--------

MoveIt2 Action Server for Dobot robots.

Launch Sequence
---------------

.. code-block:: text

   ros2 launch dobot_moveit
       -> dobot_moveit.launch.py
       -> Reads DOBOT_TYPE environment
       -> dobot-joint.launch.py + Model-Specific Launch

MoveIt Nodes
------------

- **joint_states** (State)
- **action_move_server** (MoveIt Action)

Usage
-----

.. code-block:: bash

   ros2 launch dobot_moveit dobot_moveit.launch.py
   DOBOT_TYPE=cr5 ros2 launch dobot_moveit dobot_moveit.launch.py
