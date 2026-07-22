This document is a declaration of software quality for the `px4_ros2_cpp` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `px4_ros2_cpp` Quality Declaration

The package `px4_ros2_cpp` claims to be in the **Quality Level 3** category.

Unlike an interface-only package, `px4_ros2_cpp` is a C++ library with a public API, gtest unit and integration tests, enforced static analysis (`clang-tidy` with `WarningsAsErrors: '*'`), enforced formatting (`clang-format`), and `ament_lint_common` linting. It therefore satisfies the testing and static-analysis requirements of Quality Level 3 with concrete tests and tools rather than marking them not applicable. It does not claim Quality Level 2 because line coverage is not measured against a policy gate [4.iii], there are no performance regression tests [4.iv], and, per requirement [5], the package has a direct runtime dependency (`px4_msgs`) that is itself at Quality Level 3, which caps this package at Quality Level 3 regardless of the other evidence. The rationales, notes, and caveats for the claim are organized below by each requirement listed in the [Package Requirements for Quality Level 3 in REP-2004](https://www.ros.org/reps/rep-2004.html), and each section states its evidence in this repository.

## Version Policy [1]

### Version Scheme [1.i]

`px4_ros2_cpp` uses `semver` in the sense of the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning). Version numbers track the corresponding [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) release line and are kept in step with [`px4_msgs`](https://github.com/PX4/px4_msgs), rather than following an independent library version history. The in-tree version in [package.xml](package.xml) is authoritative and is what `bloom` reads when cutting a release, as documented in the [README](../README.md#versioning) and the [CHANGELOG](CHANGELOG.rst).

### Version Stability [1.ii]

`px4_ros2_cpp` is at a stable version, i.e. `>= 1.0.0`, tracking the PX4 1.17 release line. The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

The public API is the set of headers installed from [`include/px4_ros2`](include/px4_ros2), exported through `ament_export_targets` and `install(DIRECTORY include/px4_ros2 ...)` in [CMakeLists.txt](CMakeLists.txt). All symbols intended for public use live in the `px4_ros2` namespace under that include tree. The API is documented with Doxygen (see [3.ii]).

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`px4_ros2_cpp` does not break its public API within a released ROS 2 distribution. Each PX4 release line is tracked on a dedicated `release/X.Y` branch (for example `release/1.17`, `release/1.16`), as documented in the [README](../README.md#compatibility-matrix); API-breaking work happens only on `main` and on unreleased branches, and a given ROS 2 distribution is served from the matching `release/X.Y` branch.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`px4_ros2_cpp` is a compiled shared library (`BUILD_SHARED_LIBS ON` in [CMakeLists.txt](CMakeLists.txt)) and is therefore subject to ABI breaks. ABI stability is maintained within a released ROS 2 distribution by making ABI-affecting changes only on the development (`main`) and unreleased branches; the per-line `release/X.Y` branches receive only backwards-compatible fixes.

## Change Control Process [2]

`px4_ros2_cpp` follows the recommended guidelines for ROS packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process).

### Change Requests [2.i]

All changes occur through a pull request, following the [contributing guidelines](../CONTRIBUTING.md) and the [pull request template](../.github/PULL_REQUEST_TEMPLATE.md).

### Contributor Origin [2.ii]

This package uses the [Developer Certificate of Origin](https://developercertificate.org/) as its confirmation of contributor origin policy. Contributors sign off their work with `git commit -s`, as described in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests require peer review before merging. Reviewers are assigned through the [CODEOWNERS](../.github/CODEOWNERS) file.

### Continuous Integration [2.iv]

All pull requests must pass CI. The build matrix is configured in [`.github/workflows/build.yml`](../.github/workflows/build.yml) and builds `px4_ros2_cpp` and runs its gtest unit tests (`px4_ros2_cpp_unit_tests`) against every maintained ROS 2 distribution (Humble, Jazzy, Kilted, Rolling, and Lyrical in a container), plus Windows and macOS (via RoboStack) for the C++ library. Linting and formatting run in a separate gate configured in [`.github/workflows/lint.yml`](../.github/workflows/lint.yml).

### Documentation Policy [2.v]

Pull requests that change the public API are expected to update the Doxygen documentation of the affected headers accordingly, as described in [CONTRIBUTING](../CONTRIBUTING.md).

## Documentation [3]

### Feature Documentation [3.i]

`px4_ros2_cpp` provides tooling to write PX4 external modes, send setpoints, and read vehicle state from ROS 2. Features are documented in the [README](../README.md), in the runnable examples under [`examples/cpp`](../examples/cpp), and in the [PX4 ROS 2 Interface Library user guide](https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html).

### Public API Documentation [3.ii]

The public headers carry Doxygen comments and are rendered to the published [C++ API reference](https://auterion.github.io/px4-ros2-interface-lib/topics.html). The Doxygen configuration is in [Doxyfile](../Doxyfile) (its `INPUT` is `px4_ros2_cpp/include`) and the site is generated with [`scripts/run-doxygen.sh`](../scripts/run-doxygen.sh).

### License [3.iii]

The license for `px4_ros2_cpp` is BSD-3-Clause, declared in the [package.xml](package.xml). The full license text is in [LICENSE](../LICENSE). Every C++ source and header carries an `SPDX-License-Identifier: BSD-3-Clause` header, and `ament_copyright` (enabled through `ament_lint_common` in the `BUILD_TESTING` block of [CMakeLists.txt](CMakeLists.txt)) runs as a test to confirm the headers are present.

### Copyright Statement [3.iv]

The copyright holder is the PX4 Development Team, stated in the SPDX headers of every source file and in [LICENSE](../LICENSE).

## Testing [4]

### Feature Testing [4.i]

`px4_ros2_cpp` has gtest tests exercising its features. Unit tests (target `px4_ros2_cpp_unit_tests`, sources under [`test/unit`](test/unit)) cover navigation interfaces, mission execution, modes, shared subscriptions, the vehicle command sender, VTOL handling, frame conversions, geodesic and geometry utilities, and the map projection implementation. Integration tests under [`test/integration`](test/integration) cover arming checks, global and local navigation, modes, mode executors, missions, the home-position setter, and overrides against a running autopilot. The test targets are registered in the `BUILD_TESTING` block of [CMakeLists.txt](CMakeLists.txt).

### Public API Testing [4.ii]

The unit and integration tests link against the installed library target and drive the public `px4_ros2` API directly, so the public API is exercised by the tests described in [4.i].

### Coverage [4.iii]

Partially met. The package ships unit and integration tests, but line coverage is not measured against a numeric policy gate in CI, so no coverage percentage is claimed here. This is one of the reasons the package claims Quality Level 3 rather than a higher level.

### Performance [4.iv]

Not met. There are no performance or timing regression tests. This is a further reason the package does not claim a higher quality level.

### Linters and Static Analysis [4.v]

`px4_ros2_cpp` enforces the standard linters plus additional static analysis. `ament_lint_auto` and `ament_lint_common` run through the `BUILD_TESTING` block of [CMakeLists.txt](CMakeLists.txt). The compiler is run with `-Wall -Wextra -Wpedantic -Werror`. `clang-tidy` runs with `WarningsAsErrors: '*'` per [.clang-tidy](../.clang-tidy) and is invoked over the project by [`scripts/run-clang-tidy-on-project.sh`](../scripts/run-clang-tidy-on-project.sh). Formatting is enforced by `clang-format` per [.clang-format](../.clang-format). All of these, together with the Python and file-hygiene hooks, run through [`pre-commit`](../.pre-commit-config.yaml) and in the [lint workflow](../.github/workflows/lint.yml).

## Dependencies [5]

`px4_ros2_cpp` has the following direct runtime ROS dependencies:

* `px4_msgs` (Quality Level 3)
* `rclcpp` (Quality Level 1, a ROS 2 Core package)
* `ament_index_cpp`

It also depends on the `Eigen3` linear-algebra library (declared via `eigen` and `eigen3_cmake_module`), which is an external, widely-used system dependency and is not classified under REP-2004. Because the runtime dependency `px4_msgs` is at Quality Level 3, requirement [5] caps `px4_ros2_cpp` at Quality Level 3. The package additionally has "buildtool" dependencies (`ament_cmake`, `eigen3_cmake_module`) and a build-time codegen dependency on `python3-empy`, which do not affect the resulting quality, and "test" dependencies (`ament_lint_auto`, `ament_lint_common`, `ament_cmake_gtest`) used only to test the package itself.

## Platform Support [6]

`px4_ros2_cpp` is built and unit-tested on every maintained ROS 2 distribution through CI: Humble, Jazzy, Kilted, and Rolling on native GitHub-hosted runners, Lyrical in the `ros:lyrical` container, and Jazzy on Windows and on macOS (via RoboStack). The matrix is defined in [`.github/workflows/build.yml`](../.github/workflows/build.yml) and summarized in the [README compatibility matrix](../README.md#compatibility-matrix).

## Security [7]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html). See [SECURITY.md](../SECURITY.md).
