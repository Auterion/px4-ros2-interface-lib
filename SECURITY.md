# Security Policy

This project follows the ROS 2 vulnerability disclosure guidance in [REP-2006](https://www.ros.org/reps/rep-2006.html).

## Scope

`px4-ros2-interface-lib` provides the `px4_ros2_cpp` C++ library and its `px4_ros2_py` Python bindings, a companion-computer control interface that runs alongside a PX4 flight controller and exchanges setpoints, modes, and telemetry over ROS 2 using the [`px4_msgs`](https://github.com/PX4/px4_msgs) definitions. This policy covers the library source, its build and packaging, and the CI tooling maintained in this repository. Vulnerabilities in PX4 firmware itself, or in the message definitions carried by `px4_msgs`, should be reported to those projects (see below).

## Supported Versions

Releases track the PX4 release line. Security fixes are provided for the versions below.

| Version | PX4 release line | Supported          |
| ------- | ---------------- | ------------------ |
| 1.18.x  | v1.18 (main)     | :white_check_mark: |
| 1.17.x  | v1.17 (stable)   | :white_check_mark: |
| 1.16.x  | v1.16            | :white_check_mark: |
| < 1.16  | older            | :x:                |

## Reporting a Vulnerability

Please **do not** open a public GitHub issue for security-sensitive reports.

Instead, use one of the following private channels:

- Open a [private security advisory][advisory] on this repository (**Security -> Report a vulnerability**), or
- Contact the maintainers listed in [`px4_ros2_cpp/package.xml`](px4_ros2_cpp/package.xml).

For vulnerabilities affecting PX4 firmware more broadly, follow the [PX4 security policy](https://github.com/PX4/PX4-Autopilot/security/policy). For issues in the message and service definitions, follow the [`px4_msgs` security policy](https://github.com/PX4/px4_msgs/security/policy).

When reporting, please include:

- A description of the vulnerability and its potential impact.
- Steps to reproduce or a proof of concept.
- The affected branch / version (e.g. `main`, `release/1.17`).

We will acknowledge your report as soon as possible and keep you informed of the progress towards a fix.

[advisory]: https://github.com/Auterion/px4-ros2-interface-lib/security/advisories/new
