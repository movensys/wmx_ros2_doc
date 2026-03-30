Install WMX3 Runtime
====================

The WMX3 Runtime (motion control engine) must be installed before building or
running the WMX ROS2 packages. MOVENSYS provides pre-built installer packages
for Ubuntu 22.04 and Ubuntu 24.04.

Prerequisites
-------------

- Ubuntu 22.04 LTS (Jammy) or Ubuntu 24.04 LTS (Noble)
- Internet access to the MOVENSYS download server
- ``sudo`` privileges

Download the WMX3 Installer
----------------------------

Select the archive that matches your Ubuntu version.

.. note::

   The download page may prompt for credentials. Use **guest** for both
   username and password.

Ubuntu 22.04
^^^^^^^^^^^^^

**Download URL:**

.. code-block:: text

   http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu22.04_linux5.19.0_rt10.zip

Download via command line:

.. code-block:: bash

   wget --user=guest --password=guest \
        http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu22.04_linux5.19.0_rt10.zip

Ubuntu 24.04
^^^^^^^^^^^^^

**Download URL:**

.. code-block:: text

   http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu24.04_linux6.15.2_rt2.zip

Download via command line:

.. code-block:: bash

   wget --user=guest --password=guest \
        http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu24.04_linux6.15.2_rt2.zip

Install WMX3
------------

Once the archive is downloaded, extract it and run the installer.

Ubuntu 22.04
^^^^^^^^^^^^^

.. code-block:: bash

   unzip Ubuntu22.04_linux5.19.0_rt10.zip
   cd Ubuntu22.04_linux5.19.0_rt10
   sudo dpkg -i *wmx3-installer.deb

Ubuntu 24.04
^^^^^^^^^^^^^

.. code-block:: bash

   unzip Ubuntu24.04_linux6.15.2_rt2.zip
   cd Ubuntu24.04_linux6.15.2_rt2
   sudo dpkg -i *wmx3-installer.deb

The installer places the WMX3 runtime at ``/opt/wmx3/``.

Verify the Installation
------------------------

After installation, confirm that the required headers and libraries are present:

.. code-block:: bash

   ls /opt/wmx3/include/WMX3Api.h
   ls /opt/wmx3/lib/libwmx3api.so

Both files must exist before proceeding to the next step. If either is missing,
re-run the installer or contact your MOVENSYS representative.

.. note::

   The WMX3 runtime requires a **PREEMPT_RT real-time kernel**. The download
   archive includes the matching pre-built kernel packages and an NVIDIA driver
   fix script (``fix-nvidia-rt.sh``) if you are running an NVIDIA GPU.
