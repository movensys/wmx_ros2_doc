dobot_description
=================

Overview
--------

This package contains URDF xacros and mesh assets for Dobot robots.

Package Structure
-----------------

.. code-block:: text

   dobot_description/
   ├── launch/
   ├── meshes/
   ├── urdf/
   ├── rviz/
   ├── CMakeLists.txt
   └── package.xml

Visualizing the Robot
---------------------

.. code-block:: bash

   ros2 launch dobot_description view_robot.launch.py model:=cr5
