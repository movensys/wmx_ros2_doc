Install Dependencies
====================

Source Build
------------

.. code-block:: bash

   mkdir -p ~/wmx_ws/src
   cd ~/wmx_ws/src

   git clone https://github.com/YOUR_ORG/dobot_moveit.git
   git clone https://github.com/YOUR_ORG/dobot_robot_interface.git
   git clone https://github.com/YOUR_ORG/dobot_description.git

   cd ~/wmx_ws
   sudo apt update
   rosdep update
   rosdep install --ignore-src --from-paths src -y

   colcon build --symlink-install
   source install/setup.bash
