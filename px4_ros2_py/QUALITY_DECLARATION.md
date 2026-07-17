This document is a declaration of software quality for the `px4_ros2_py` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `px4_ros2_py` Quality Declaration

The package `px4_ros2_py` claims to be in the **Quality Level 4** category.

`px4_ros2_py` is a [pybind11](https://pybind11.readthedocs.io/) wrapper that exposes a subset of the [`px4_ros2_cpp`](../px4_ros2_cpp/QUALITY_DECLARATION.md) C++ library to Python. It inherits the quality of `px4_ros2_cpp` (declared at Quality Level 3) for everything it re-exports, but it claims one level lower because the Python surface is explicitly incomplete (only part of the C++ API is bound) and the package does not yet ship dedicated feature, unit, or coverage tests of its own. The rationales, notes, and caveats for the Quality Level 4 claim are given below, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`px4_ros2_py` uses `semver` according to the recommendation for ROS packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning). Its version tracks the PX4 release line rather than an independent scheme, so it is released in lockstep with `px4_ros2_cpp` and the matching `px4_msgs` set.

### Version Stability [1.ii]

`px4_ros2_py` is at a stable version, i.e. `>= 1.0.0`. The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst). Users should note that, as stated in the [README](../README.md), the Python bindings are not yet complete: the set of bound symbols is a subset of the `px4_ros2_cpp` public API and is expected to grow, so additive changes to the Python surface are anticipated within this version line.

### Public API Declaration [1.iii]

The public API of `px4_ros2_py` is the `px4_ros2` Python module and its submodules, which are produced by the pybind11 bindings registered in [`src/px4_ros2/bind_main.cpp`](src/px4_ros2/bind_main.cpp). It currently exposes the `geometry`, `components`, `control`, and `odometry` submodules, wrapping the corresponding parts of `px4_ros2_cpp`. The `px4_ros2` Python package re-exports the compiled module via [`px4_ros2/__init__.py`](px4_ros2/__init__.py). Only the symbols bound in these translation units are part of the public API; the underlying C++ types and behaviour are documented by [`px4_ros2_cpp`](../px4_ros2_cpp/QUALITY_DECLARATION.md).

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`px4_ros2_py` will not break its bound public API within a released ROS 2 distribution; changes that would break the API are made only on the development (`main`) branch. Each PX4 release line is tracked on a dedicated `release/*` branch, as documented in the [README](../README.md). Because the bindings are incomplete, new symbols may be added within a distribution, but existing ones are not removed or altered in a breaking way once released.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`px4_ros2_py` builds a compiled C-extension module and therefore is subject to ABI concerns. As with `px4_ros2_cpp`, ABI stability is maintained within a released ROS 2 distribution by only making ABI-affecting changes on the development (`main`) and unreleased branches.

## Change Control Process [2]

`px4_ros2_py` is developed in the same repository as `px4_ros2_cpp` and follows the same change control process, described in the repository [contributing guidelines](../CONTRIBUTING.md).

### Change Requests [2.i]

All changes occur through a pull request.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests are built in CI, configured in [`.github/workflows/build.yml`](../.github/workflows/build.yml). The `px4_ros2_py` module is compiled and its Python package is installed on the supported Linux ROS 2 distributions; the pybind11 build and import therefore exercise the bindings on every change. Some platforms (for example Windows and macOS) may build only `px4_ros2_cpp` where the pybind11 or `px4_msgs` codegen prerequisites are not available; this is reflected in the CI package selection and is a reason the Python package claims a lower quality level than the C++ package.

### Documentation Policy [2.v]

Pull requests that change the public API are expected to update the documentation accordingly. Because `px4_ros2_py` mirrors `px4_ros2_cpp`, the authoritative behavioural documentation lives with the C++ package and its API reference.

## Documentation [3]

### Feature Documentation [3.i]

`px4_ros2_py` provides Python bindings for writing PX4 external modes from a companion computer. Usage is documented in the repository [README](../README.md), in the runnable Python examples under [`examples/python`](../examples/python), and in the [PX4 ROS 2 Interface Library guide](https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html). The incompleteness of the bindings is stated in the README so that users know which functionality is available from Python.

### Public API Documentation [3.ii]

The Python API reference is published at [auterion.github.io/px4-ros2-interface-lib/python](https://auterion.github.io/px4-ros2-interface-lib/python/px4_ros2_py.html). The bound entities carry docstrings defined in the pybind11 sources, and the semantics of each wrapped entity are documented by the corresponding [`px4_ros2_cpp`](../px4_ros2_cpp/QUALITY_DECLARATION.md) API. Not every bound symbol is documented to the same depth as the C++ API, which is consistent with the Quality Level 4 claim.

### License [3.iii]

The license for `px4_ros2_py` is BSD-3-Clause, and a summary can be found in the [package.xml](package.xml). The full license text is in [LICENSE](../LICENSE). Every source file in this package carries an SPDX `BSD-3-Clause` license header, and the presence and formatting of those headers is checked by the shared [`pre-commit`](../.pre-commit-config.yaml) configuration and in CI.

### Copyright Statement [3.iv]

The copyright holders are listed in the per-file license headers and in [LICENSE](../LICENSE).

## Testing [4]

`px4_ros2_py` declares the `ament_cmake_pytest` and `python3-pytest` test dependencies in its [package.xml](package.xml), providing the harness for Python tests. It does not yet ship dedicated feature, unit, or coverage tests, which is the primary reason for the Quality Level 4 claim. The behaviour reachable through the bindings is covered by the tests of the wrapped library, [`px4_ros2_cpp`](../px4_ros2_cpp/QUALITY_DECLARATION.md).

### Feature Testing [4.i]

There are currently no dedicated feature tests for the Python bindings. The wrapped features are feature-tested at the C++ level in `px4_ros2_cpp`. Building and importing the compiled module in CI verifies that the bindings load.

### Public API Testing [4.ii]

There are currently no dedicated Python API tests. New tests are expected to be added as the binding surface grows toward completeness; the pytest harness is already declared so such tests can be added without further packaging changes.

### Coverage [4.iii]

Code coverage is not measured for `px4_ros2_py`. Coverage of the underlying functionality is reported by `px4_ros2_cpp`.

### Performance [4.iv]

There are no performance tests. The bindings are thin pass-throughs to `px4_ros2_cpp`, whose performance characteristics govern runtime behaviour.

### Linters and Static Analysis [4.v]

`px4_ros2_py` uses and passes the standard linters for its languages. Python sources are linted and formatted with `ruff` (`ruff-check` and `ruff-format`), configured via [`ruff.toml`](../ruff.toml). The C++ binding sources are formatted with `clang-format`. Repository-wide checks (codespell, YAML and XML validation, shellcheck, markdownlint, and `package.xml` schema validation) also apply. All of these are enforced through the shared [`pre-commit`](../.pre-commit-config.yaml) configuration and run in CI.

## Dependencies [5]

`px4_ros2_py` depends most directly on [`px4_ros2_cpp`](../px4_ros2_cpp/QUALITY_DECLARATION.md), which it wraps; its quality is bounded by that of `px4_ros2_cpp` (declared at Quality Level 3). It has the following runtime dependencies:

* `px4_ros2_cpp` (Quality Level 3, see its [Quality Declaration](../px4_ros2_cpp/QUALITY_DECLARATION.md); transitively depends on `px4_msgs`)
* `rclcpp`
* `rclpy`
* `ament_index_python`

Its build additionally requires `pybind11-dev`. It has "buildtool" dependencies, which do not affect the resulting quality, and "test" dependencies (`ament_cmake_pytest`, `python3-pytest`) that are only used to test the package itself. No dependency is claimed at a higher quality level than this package.

## Platform Support [6]

`px4_ros2_py` is built and its module import is exercised on the Linux tier 1 platforms described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers) via CI. As noted under Continuous Integration, the Python package may be excluded on platforms where the pybind11 or `px4_msgs` codegen prerequisites are unavailable, so its platform coverage is narrower than that of `px4_ros2_cpp`.

## Security [7]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html). See [SECURITY.md](../SECURITY.md).
