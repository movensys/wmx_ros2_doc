.. `:tocdepth: 1` means showing only title in contents

:tocdepth: 1

WMX3 on ARM-based PC
=========================

Installing WMX3 on an ARM machine is for real-world scenarios.
Before moving to real motor control testing on the IPC,
it is recommended to :bi:`verify basic behavior in a simulator` (WMX3, Isaac Sim, Gazebo, etc.).


Download the WMX3 Installer
----------------------------

Unzip ``wmx3_arm64_installers.zip`` from the download site.

.. code-block:: bash

   wget --user=guest --password=guest http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/wmx3_arm64_installers.zip
   unzip wmx3_arm64_installers.zip


Currently supported IPCs are listed below.

1. Advantech MIC-713, NVIDIA Jetson Orin NX on Ubuntu 20.04

.. code-block:: bash

   sudo dpkg -i 20260403_Ubuntu20.04_linux-5.10.120-rt70-jetson-orin-nx-mic-713-wmx3-installer.deb

2. Advantech MIC-733ao, NVIDIA Jetson Orin AGX on Ubuntu 22.04

.. code-block:: bash

   sudo dpkg -i 20260403_Ubuntu22.04_linux-5.15.148-rt-jetson-agx-orin-mic-733ao-wmx3-installer.deb

For the ``5.15.148-tegra`` kernel, we also recommend using ``rt_igb.ko`` from ``rt_igc_igb_5.15.148-rt-tegra.zip``:

.. code-block:: bash

   wget --user=guest --password=guest http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/rt_igc_igb_5.15.148-rt-tegra.zip
   unzip rt_igc_igb_5.15.148-rt-tegra.zip


3. Advantech MIC-743, NVIDIA Jetson Thor on Ubuntu 24.04

.. code-block:: bash

   sudo dpkg -i 20260403_Ubuntu24.04_linux-6.8.12-rt-jetson-thor-mic-743-wmx3-installer.deb

The installer places the WMX3 runtime at ``/opt/wmx3/``.


Other Environments
^^^^^^^^^^^^^^^^^^^

Our product depends on Network Interface Card (NIC) drivers, not on specific IPCs.
If you share Ubuntu kernel version and NIC driver information to issue boards (https://github.com/movensys/wmx-ros2/issues),
we can support WMX3 installation for your environment.

Check your kernel version:

.. code-block:: bash

   uname -a

Check your NIC drivers:

.. code-block:: bash

   lspci -v


Verify the Installation
------------------------

After installation, confirm that the required headers and libraries are present:

.. code-block:: bash

   ls /opt/wmx3/include/WMX3Api.h
   ls /opt/wmx3/lib/libwmx3api.so

Both files must exist before proceeding to the next step. If either is missing,
re-run the installer or contact your MOVENSYS representative.
