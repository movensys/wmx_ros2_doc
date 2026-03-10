Integration Scenarios
=====================

WMX ROS2 supports multiple motion planning backends and can be extended with
custom planners and applications. Use the decision tree below to find the right
integration for your use case, then follow the corresponding guide.

.. mermaid::
   :caption: Integration — which scenario fits your use case?
   :zoom:

   %%{init: {"theme": "base", "themeVariables": {"primaryColor": "#1a73e8", "primaryTextColor": "#fff", "primaryBorderColor": "#1558b0", "lineColor": "#555"}}}%%
   flowchart TD
       START(["What do I need?"])

       Q1{{"NVIDIA GPU<br/>with CUDA<br/>available?"}}
       Q2{{"Need vision or<br/>3D obstacle<br/>avoidance?"}}
       Q3{{"Writing a custom<br/>planning algorithm?"}}
       Q4{{"Building a<br/>standalone ROS2 app?"}}

       R_MV["MoveIt2 Integration<br/>─────────────────<br/>Standard motion planning<br/>OMPL / CHOMP / Pilz<br/>Works on any hardware"]
       R_CU["Isaac cuMotion<br/>─────────────────<br/>GPU-accelerated planning<br/>AprilTag pick & place<br/>NvBlox obstacle avoidance"]
       R_CP["Custom Planner<br/>─────────────────<br/>Plug in your own<br/>planning algorithm<br/>via MoveIt2 plugin API"]
       R_CA["Custom Application<br/>─────────────────<br/>Call WMX services directly<br/>Build your own ROS2 node<br/>No MoveIt2 required"]

       START --> Q1
       Q1 -->|"Yes"| Q2
       Q1 -->|"No"| Q3

       Q2 -->|"Yes"| R_CU
       Q2 -->|"No"| R_MV

       Q3 -->|"Yes"| R_CP
       Q3 -->|"No"| Q4

       Q4 -->|"Yes"| R_CA
       Q4 -->|"No"| R_MV

       style R_MV fill:#34a853,color:#fff
       style R_CU fill:#1a73e8,color:#fff
       style R_CP fill:#fa7b17,color:#fff
       style R_CA fill:#9334e6,color:#fff
       style START fill:#1a1a1a,color:#fff

Available integrations:

- **MoveIt2** -- Standard ROS2 motion planning via ``FollowJointTrajectory``
- **Isaac cuMotion** -- NVIDIA GPU-accelerated collision-aware planning
- **Custom Planner** -- Integrate your own planning algorithm
- **Custom Application** -- Build standalone ROS2 apps using WMX services

.. toctree::
   :maxdepth: 2

   moveit2_integration
   cumotion_integration
   custom_planner
   custom_application
