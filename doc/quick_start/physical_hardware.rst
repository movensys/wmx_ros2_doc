Launching with Physical Hardware
=================================

Prerequisites
-------------

- Dobot robot powered on
- Ethernet cable connected between PC and Dobot controller
- Robot IP address configured

1. Network Configuration
-------------------------

TODO: Add network setup

2. Enable Robot
----------------

.. code-block:: bash

   ros2 launch dobot_robot_interface dobot_interface.launch.py robot_ip:=192.168.1.6
   ros2 service call /EnableRobot dobot_msgs/srv/EnableRobot

3. Launch MoveIt2
------------------

.. code-block:: bash

   ros2 launch dobot_moveit dobot_moveit.launch.py
