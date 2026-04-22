.. `:tocdepth: 1` means showing only title in contents

:tocdepth: 1

WMX3 on x86-based PC
=========================

We recommend to :bi:`verify basic behavior in a simulator` (WMX3, Isaac Sim, Gazebo, etc.)
before moving to real motor control testing on robot control machine.


Download the WMX3 Installer
----------------------------

Select the archive that matches your Ubuntu version.

Ubuntu 22.04
^^^^^^^^^^^^^
.. code-block:: bash

   wget --user=guest --password=guest http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu22.04_linux5.19.0_rt10.zip

Ubuntu 24.04
^^^^^^^^^^^^^
.. code-block:: bash

   wget --user=guest --password=guest http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu24.04_linux6.15.2_rt2.zip

.. note::

   If the wget command fails, download the archive directly via the URL using a web browser.

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Ubuntu version
     - Download via URL
   * - Ubuntu 22.04
     - `Ubuntu22.04_linux5.19.0_rt10.zip <http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu22.04_linux5.19.0_rt10.zip>`_
   * - Ubuntu 24.04
     - `Ubuntu24.04_linux6.15.2_rt2.zip <http://download.movensys.com:8111/webdav/WMX3_Installer/Linux/Ubuntu24.04_linux6.15.2_rt2.zip>`_
.. note::

   The download page may prompt for credentials. Use ***guest*** for both
   username and password.


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
   ls /opt/wmx3/lib/libimdll.so

Both files must exist before proceeding to the next step. If either is missing,
re-run the installer or contact your MOVENSYS representative.

.. note::

   WMX3 currently supports simulation mode only.
