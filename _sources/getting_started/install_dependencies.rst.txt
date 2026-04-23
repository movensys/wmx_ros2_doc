Install Dependencies
====================

Install the required ROS2 packages:

.. code-block:: bash

   sudo apt update
   sudo apt install -y ros-${ROS_DISTRO}-graph-msgs \
                       ros-${ROS_DISTRO}-moveit* \
                       ros-${ROS_DISTRO}-ros2-control \
                       ros-${ROS_DISTRO}-ros2-controllers \
                       ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
   sudo apt install -y python3-colcon-common-extensions python3-rosdep

Verify WMX3 Runtime Installation
----------------------------------

The WMX3 Runtime must be installed at ``/opt/wmx3/`` before building the
workspace. See :doc:`install_wmx3` for download and installation instructions.

To confirm the installation is present:

.. code-block:: bash

   ls /opt/wmx3/include/WMX3Api.h
   ls /opt/wmx3/lib/libwmx3api.so

Both files must exist before proceeding.
