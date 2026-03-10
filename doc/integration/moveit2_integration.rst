MoveIt2 Motion Planning
========================

Overview
--------

MoveIt2 is the primary motion planning integration for the WMX ROS2
application. It provides collision-aware trajectory planning, and the
``follow_joint_trajectory_server`` node executes the planned trajectories
on real hardware via the WMX cubic spline engine.

Architecture
------------

.. mermaid::
   :caption: MoveIt2 integration — end-to-end data flow
   :zoom:

   %%{init: {"theme": "base", "themeVariables": {"primaryColor": "#1a73e8", "primaryTextColor": "#fff", "primaryBorderColor": "#1558b0", "lineColor": "#555"}}}%%
   flowchart TB
       UA["User / RViz2 Plugin<br/>(set target pose or joint goal)"]

       subgraph WMX_ROS["WMX ROS2 Nodes"]
           MS["manipulator_state<br/>Publishes /joint_states @ 500 Hz"]
           FJT["follow_joint_trajectory_server<br/>WMX3 cubic spline  (max 1000 waypoints)"]
       end

       subgraph MV2["MoveIt2"]
           MV["Motion Planner<br/>(OMPL / CHOMP / Pilz)"]
           CS["Current State Monitor<br/>Reads /joint_states"]
       end

       subgraph HW["Hardware"]
           AM["AdvancedMotion API<br/>StartCSplinePos()"]
           EC["EtherCAT Bus"]
           SD["Servo Drives J1–J6"]
           ROB["Dobot CR3A"]
       end

       UA -->|"Plan & Execute request"| MV
       CS -->|"Current joint positions"| MV
       MV -->|"FollowJointTrajectory<br/>action goal"| FJT
       MS -->|"/joint_states"| CS

       FJT -->|"CSplinePos trajectory<br/>via WMX API"| AM
       AM -->|"Real-time spline"| EC
       EC --> SD --> ROB
       ROB -->|"Encoder feedback"| MS
       FJT -->|"Result: success / error_code"| MV

       style WMX_ROS fill:#e8f0fe,stroke:#1a73e8,color:#1a1a1a
       style MV2 fill:#e6f4ea,stroke:#34a853,color:#1a1a1a
       style HW fill:#fce8e6,stroke:#ea4335,color:#1a1a1a

Prerequisites
-------------

MoveIt2 must be installed as part of the workspace dependencies:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-moveit*

See :doc:`../getting_started/index` for the complete dependency list.

Configuration
-------------

The ``follow_joint_trajectory_server`` node must be configured with the
correct action name that MoveIt2 expects. This is set in the config YAML:

.. code-block:: yaml

   follow_joint_trajectory_server:
     ros__parameters:
       joint_number: 6
       joint_trajectory_action: /movensys_manipulator_arm_controller/follow_joint_trajectory

The action name must match the controller configuration in the MoveIt2
setup for your robot.

The ``manipulator_state`` node must publish to ``/joint_states`` so MoveIt2
can read the current robot state:

.. code-block:: yaml

   manipulator_state:
     ros__parameters:
       encoder_joint_topic: /joint_states

Usage
-----

1. Launch the WMX ROS2 manipulator nodes (see
   :doc:`../getting_started/index`):

   .. code-block:: bash

      sudo --preserve-env=PATH \
           --preserve-env=AMENT_PREFIX_PATH \
           --preserve-env=COLCON_PREFIX_PATH \
           --preserve-env=PYTHONPATH \
           --preserve-env=LD_LIBRARY_PATH \
           --preserve-env=ROS_DISTRO \
           --preserve-env=ROS_VERSION \
           --preserve-env=ROS_PYTHON_VERSION \
           --preserve-env=ROS_DOMAIN_ID \
           --preserve-env=RMW_IMPLEMENTATION \
           bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
                    source ~/wmx_ros2_ws/install/setup.bash && \
                    ros2 launch wmx_ros2_package \
                      wmx_ros2_cr3a_manipulator.launch.py \
                      use_sim_time:=false"

2. Launch MoveIt2 with your robot's MoveIt configuration package.

3. Plan and execute trajectories through the MoveIt2 interface (RViz plugin
   or programmatic API).

Trajectory Execution Details
-----------------------------

When MoveIt2 sends a trajectory, the ``follow_joint_trajectory_server``
processes it as described in the :doc:`../api_reference/ros2_actions` page:

- Maximum 1000 waypoints per trajectory
- Cubic spline interpolation via ``AdvancedMotion::StartCSplinePos()``
- The server blocks until motion completes
- Returns ``error_code = 0`` on success

