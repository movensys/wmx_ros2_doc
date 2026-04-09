.. `:tocdepth: 1` means showing only title in contents

:tocdepth: 1

WMX3 on Industrial PC
=========================

Installing WMX3 on an Industrial PC (IPC) is for real-world scenarios.
Before moving to real motor control testing on the IPC,
it is recommended to :bi:`verify basic behavior in a simulator` (WMX3, Isaac Sim, Gazebo, etc.).


Download the WMX3 Installer
----------------------------

Unzip ``wmx3_arm64_installers.zip`` from the download site.

.. code-block:: bash

   unzip wmx3_arm64_installers.zip


Currently supported IPCs are listed below.

1. MIC-713, NVIDIA Jetson Orin NX on Ubuntu 20.04

.. code-block:: bash

   sudo dpkg -i 20260403_Ubuntu20.04_linux-5.10.120-rt70-jetson-orin-nx-mic-713-wmx3-installer.deb

2. MIC-733ao, NVIDIA Jetson Orin AGX on Ubuntu 22.04

.. code-block:: bash

   sudo dpkg -i 20260403_Ubuntu22.04_linux-5.15.148-rt-jetson-agx-orin-mic-733ao-wmx3-installer.deb

For the ``5.15.148-tegra`` kernel, we also recommend using ``rt_igb.ko`` from ``rt_igc_igb_5.15.148-rt-tegra.zip``:

.. code-block:: bash

   sudo cp rt_igb.ko /lib/modules/$(uname -r)/kernel/drivers/net/
   sudo depmod
   sudo modprobe rt_igb


3. MIC-743, NVIDIA Jetson Thor on Ubuntu 24.04

.. code-block:: bash

   sudo dpkg -i 20260403_Ubuntu24.04_linux-6.8.12-rt-jetson-thor-mic-743-wmx3-installer.deb

The installer places the WMX3 runtime at ``/opt/wmx3/``.


Other Environments
^^^^^^^^^^^^^^^^^^^

Our product depends on Network Interface Card (NIC) drivers, not on specific IPCs.
If you email your Ubuntu kernel version and NIC driver information to us (sjhwang@movensys.com),
we can support WMX3 installation for your environment.

Check your kernel version:

.. code-block:: bash

   uname -r

Check your NIC drivers:

.. code-block:: bash

   lspci


Verify the Installation
------------------------

After installation, confirm that the required headers and libraries are present:

.. code-block:: bash

   ls /opt/wmx3/include/WMX3Api.h
   ls /opt/wmx3/lib/libwmx3api.so

Both files must exist before proceeding to the next step. If either is missing,
re-run the installer or contact your MOVENSYS representative.
