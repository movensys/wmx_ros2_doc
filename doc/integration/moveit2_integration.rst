MoveIt2 Motion Planning
========================

Overview
--------

The primary integration scenario: MoveIt2 for motion planning
and trajectory execution on Dobot collaborative robots.

Architecture
------------

.. code-block:: text

   User Application
       -> ROS2_MoveIt_Action_Server
           -> action_move_server
               -> ROS2_Dobot_Robot_Interface
                   -> ServoJ / EnableRobot
                       -> TCP/IP -> Dobot Controller -> Robot

Setup
-----

TODO: Add setup instructions

Usage Example
-------------

TODO: Add usage example
