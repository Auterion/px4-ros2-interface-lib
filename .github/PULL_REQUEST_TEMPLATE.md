<!--
Thanks for contributing to px4-ros2-interface-lib!

This library provides the px4_ros2_cpp C++ interface and its px4_ros2_py Python bindings for controlling PX4 from a companion computer over ROS 2. See CONTRIBUTING.md before you start.
-->

## Description

<!-- Describe your changes in detail. What problem does this PR solve? -->

## Type of change

<!-- Put an `x` in the boxes that apply. -->

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing behavior to change)
- [ ] Build / CI / packaging change
- [ ] Documentation update

## Related issues

<!-- Link any related issues, e.g. "Fixes #123" or "Relates to #123". -->

## Checklist

- [ ] I have read the [CONTRIBUTING](../CONTRIBUTING.md) guide.
- [ ] `pre-commit run --all-files` passes (clang-format, clang-tidy, ruff).
- [ ] The affected packages build (`colcon build --packages-up-to px4_ros2_cpp px4_ros2_py`) and unit tests pass (`colcon test`; `px4_ros2_cpp` gtest unit tests and `px4_ros2_py` pytest).
- [ ] New or changed behavior is covered by tests.
- [ ] My changes do not edit the vendored `px4_ros2_cpp/include/px4_ros2/third_party/` sources (those track upstream).
- [ ] I have updated the relevant `CHANGELOG.rst` and bumped versions in `package.xml` if appropriate.
- [ ] My commits follow [Conventional Commits](https://www.conventionalcommits.org/) and are signed off (`git commit -s`, DCO).
