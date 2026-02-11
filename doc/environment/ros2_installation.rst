ROS2 Installation
=================

This guide walks through installing ROS2 on Ubuntu.

.. note::

   Follow the official `ROS 2 Installation Guide <https://docs.ros.org/en/humble/Installation.html>`_
   for the Desktop Install of ROS2 Humble.

1. Set Locale
-------------

.. code-block:: bash

   locale
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

2. Setup Sources
----------------

.. code-block:: bash

   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

3. Install ROS2
----------------

.. code-block:: bash

   sudo apt update
   sudo apt upgrade
   sudo apt install ros-humble-desktop

4. Environment Setup
--------------------

.. code-block:: bash

   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc

5. Verify Installation
----------------------

.. code-block:: bash

   ros2 run demo_nodes_cpp talker
