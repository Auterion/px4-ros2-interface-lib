Changelog for package px4_ros2_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The version numbers of this package track the corresponding `PX4 Autopilot
<https://github.com/PX4/PX4-Autopilot>`_ release line rather than an
independent library semver. This package provides Python bindings that wrap
``px4_ros2_cpp`` and is released in lockstep with it. Starting with this entry
the package is released through the standard ROS 2 build-farm cycle, so the
version recorded here is authoritative and read by bloom at release time.

1.17.0 (2026-07-16)
-------------------
* Adopted the standard ROS 2 build-farm release cycle for the Python bindings.
* Aligned the package version to the PX4 Autopilot 1.17 release line.
* Packaging and release milestone only; the wrapper around ``px4_ros2_cpp`` is functionally unchanged.
* Contributors: Nuno Marques
