# Contributing

Thanks for your interest in contributing to the PX4 ROS 2 Interface Library!

This repository ships two ROS 2 ament packages that are released together and in a fixed order: `px4_ros2_cpp` (the `ament_cmake` C++ library) and `px4_ros2_py` (the pybind11 Python bindings that wrap it). `px4_ros2_cpp` build- and run-depends on `px4_msgs`, and `px4_ros2_py` build-, link- and run-depends on `px4_ros2_cpp`.

## Contributing changes

1. Fork the repository and create a topic branch off `main`.
2. Make your changes. For development, install the pre-commit hooks once with `pre-commit install` so the formatters and linters run on every commit.
3. Ensure both packages still build and their tests pass. Source your ROS 2 workspace first, then:
   ```sh
   colcon build --packages-up-to px4_ros2_py
   colcon test --packages-select px4_ros2_cpp --ctest-args -R unit_tests
   colcon test --packages-select px4_ros2_py
   colcon test-result --verbose
   ```
   The `px4_ros2_cpp` integration tests need a live FMU and are not run in CI; the unit tests (`-R unit_tests`) are.
4. Run the linters over your changes:
   ```sh
   pre-commit run --all-files
   ```
5. Open a pull request describing the change and its rationale.

### Commit conventions

Please write [Conventional Commits](https://www.conventionalcommits.org/) and sign off your work under the [Developer Certificate of Origin](https://developercertificate.org/) using `git commit -s`.

All C++ and pybind source files carry an SPDX license header (`SPDX-License-Identifier: BSD-3-Clause`, `Copyright (c) YYYY PX4 Development Team`). New source files must carry the same header; the copyright and license checks run in CI.

## Releasing

Releases are driven from the in-tree package version and cascade automatically through three GitHub Actions workflows. The pipeline mirrors the standard ROS 2 build-farm flow adopted by [`PX4/px4_msgs`](https://github.com/PX4/px4_msgs).

### The release cascade

1. [`Create GitHub release`](.github/workflows/create-release.yml) runs when a `vX.Y.Z` tag is pushed, or when a `release/**` branch is pushed whose `package.xml` version has no matching `vX.Y.Z` tag yet. It creates the tag (for the branch-bump case) and publishes a GitHub release with auto-generated notes.
2. Publishing that release triggers [`Package (.deb)`](.github/workflows/package.yml), which builds Debian packages for every supported ROS 2 distribution and architecture using the bundled builder container and attaches the `.deb` files to the release. Because `px4_msgs` is not yet on the ROS build farm, the builder container builds the `px4_msgs` dependency from source itself before building this repo's packages; `rosdep` cannot resolve it as a binary yet.
3. Publishing the release also triggers [`Release to rosdistro (bloom)`](.github/workflows/release.yml), which runs `bloom-release` to open the release pull requests against [`ros/rosdistro`](https://github.com/ros/rosdistro) so the packages are built by the official ROS build farm.

#### `RELEASE_TOKEN` is required for the cascade

A GitHub release created with the default `GITHUB_TOKEN` does not trigger downstream workflows. To let `Create GitHub release` cascade into `Package (.deb)` and `Release to rosdistro (bloom)`, add a `RELEASE_TOKEN` repository secret holding a personal access token (PAT) with permission to create releases on this repository. If `RELEASE_TOKEN` is absent the release is still created with the default token, but the packaging and bloom workflows will not run.

### Strict release order

The packages depend on each other, so releases must be cut in this order and each dependency must be released (and, for the rosdistro path, available on the build farm) before the next:

```
px4_msgs  ->  px4_ros2_cpp  ->  px4_ros2_py
```

`px4_msgs` is released from its own repository. Within this repository, always release `px4_ros2_cpp` before `px4_ros2_py`, because `px4_ros2_py` build-, link- and run-depends on `px4_ros2_cpp`.

### Honest status: the rosdistro path is currently blocked

`px4_ros2_cpp` build-depends on `px4_msgs`, and `px4_msgs` is not yet on the ROS build farm: it is `source:`-only in `ros/rosdistro` and has no `release:` block, so `bloom-release` cannot resolve it as a binary dependency. Until that changes, the rosdistro (bloom) path cannot complete a binary release of these packages.

The [`Release to rosdistro (bloom)`](.github/workflows/release.yml) workflow is written to fail safe under this constraint: it exits successfully without doing anything while the `BLOOM_GITHUB_TOKEN` secret is unset, so it never blocks a release or turns the board red. The `Package (.deb)` workflow does not depend on the build farm (it builds `px4_msgs` from source in the container), so `.deb` artifacts are produced regardless.

In short: `release.yml` no-ops until `BLOOM_GITHUB_TOKEN` is set and the `px4_msgs` dependency is resolvable on the build farm. Do not set `BLOOM_GITHUB_TOKEN` until both the one-time setup below is complete and `px4_msgs` has reached the farm.

### One-time maintainer setup

These steps must be done once by a maintainer before the rosdistro (bloom) automation is operational.

1. Create the two bloom release repositories, one per package:
   - `${RELEASE_ORG}/px4_ros2_cpp-release`
   - `${RELEASE_ORG}/px4_ros2_py-release`

   `RELEASE_ORG` is a workflow-level variable that defaults to `Auterion`. It exists so the target org can be changed in one place. Choose the org deliberately: this library currently lives under `Auterion`, and its documentation and existing `release/**` branches are published there, so `Auterion` is the natural home for the release repositories. If the packages are moved under the `PX4` org (to sit alongside `PX4/px4_msgs` and the rest of the PX4 release repositories), set the `RELEASE_ORG` repository variable to `PX4` and create the two `*-release` repositories there instead. Keep the release repositories in the same org that owns this source repository.

2. Seed a bloom track in each release repository, one per supported ROS 2 distribution (`humble`, `jazzy`, `kilted`, `rolling`, `lyrical`), using an interactive `bloom-release` with `main` as the upstream devel branch. Run this once per package and distribution, for example:
   ```sh
   bloom-release --rosdistro jazzy --new-track --track jazzy px4_ros2_cpp
   bloom-release --rosdistro jazzy --new-track --track jazzy px4_ros2_py
   ```
   When prompted, set the upstream `devel_branch` to `main` and point the track at `${RELEASE_ORG}/px4_ros2_cpp-release` (respectively `px4_ros2_py-release`). This first interactive run is also what adds the `release:` block for each package to `ros/rosdistro`. After the tracks and the `release:` blocks exist, the [`Release to rosdistro (bloom)`](.github/workflows/release.yml) workflow automates subsequent releases non-interactively.

3. Add a `BLOOM_GITHUB_TOKEN` repository secret: a token (a fine-grained PAT or a bot account token) that can push to `${RELEASE_ORG}/px4_ros2_cpp-release` and `${RELEASE_ORG}/px4_ros2_py-release` and open pull requests on `ros/rosdistro`. Setting this secret is what enables the bloom workflow; leave it unset until the tracks are seeded and `px4_msgs` is resolvable on the build farm (see the blocker note above).

4. Add the `RELEASE_TOKEN` secret described under the release cascade above.

### Cutting a release

Once the setup above is in place, a release is cut in one of two equivalent ways. Both drive the same cascade via the in-tree `package.xml` version, so keep the version in the two `package.xml` files in sync.

- Bump-on-a-release-branch: on a `release/X.Y` branch, set the `<version>` in both `px4_ros2_cpp/package.xml` and `px4_ros2_py/package.xml` to `X.Y.Z`, commit, and push the branch. `Create GitHub release` reads the version from `package.xml`, creates the `vX.Y.Z` tag if it is missing, and publishes the release.
- Tag-push: push an annotated `vX.Y.Z` tag matching the `package.xml` version. `Create GitHub release` publishes the release directly from the tag.

Publishing the release then triggers `Package (.deb)` and (once `BLOOM_GITHUB_TOKEN` is set and `px4_msgs` is on the farm) `Release to rosdistro (bloom)`.

### Versioning

Package versions follow the PX4 release line (the same versioning `px4_msgs` uses), not an independent library semver. Both packages in this repository are versioned together and are currently at `1.17.0`, matching the latest PX4 stable line. The in-tree `package.xml` version is authoritative: bloom reads it, and the release cascade is triggered from it.

## Credits and acknowledgements

The ROS 2 build-farm release pipeline documented above (the packaging, bloom and CI workflows, and this contributing guide) is a contribution from [RIIS, LLC](https://www.riis.com).
