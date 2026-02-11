dobot_robot_interface
=====================

Overview
--------

This package provides the ROS2 interface to the Dobot robot controller
via TCP/IP.

Key Components
--------------

- **main.cpp** - Entry point
- **CRRobotRos2** - Core communication class
- **CRCommanderRos2** - TCP command serialization

ROS2 Services
-------------

.. list-table::
   :header-rows: 1

   * - Service
     - Description
   * - /EnableRobot
     - Enable the robot
   * - /ServoJ
     - Joint servo command
   * - Other
     - Control, Motion, IO, Safety, Config

Publishers
----------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - /joint_states
     - sensor_msgs/JointState
   * - /robot_status
     - TODO
   * - /tool_vector
     - TODO
