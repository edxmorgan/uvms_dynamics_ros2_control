.. _Underwater_Vehicle_and_Manipulator_System_userdoc:

Underwater_Vehicle_and_Manipulator_System
==========================================

This is a ros2_control plugin providing virtual underwater vehicle and manipulator capabilities.


Hardware interface type
-----------------------

This controller exports position and velocity state interface and effort command interface


ROS 2 interface of the controller
---------------------------------


Parameters
^^^^^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

   .. tabs::

      .. group-tab:: uvms

        .. generate_parameter_library_details:: ../src/uvms_parameters.yaml
