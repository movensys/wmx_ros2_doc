Troubleshooting
===============

Connection Failed
-----------------

**Symptom:** Cannot connect to Dobot robot controller.

**Solution:**

- Check Ethernet cable connection
- Verify IP address: ``ping 192.168.1.6``

Robot Not Enabled
-----------------

**Symptom:** Motion commands are rejected.

**Solution:**

- Call ``/EnableRobot`` service first

MoveIt Planning Failed
----------------------

**Symptom:** MoveIt2 fails to find a trajectory.

**Solution:**

- Check joint limits in URDF
- Verify target pose is reachable

Build Errors
------------

**Solution:**

.. code-block:: bash

   rosdep install --ignore-src --from-paths src -y

Getting Help
------------

- GitHub Issues: https://github.com/YOUR_ORG/dobot_moveit/issues
