/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "components/mode.hpp"
#include "components/mode_executor.hpp"
#include "components/node_wrapper.hpp"
#include "control/setpoint_types/multicopter/goto.hpp"
#include "odometry/local_position.hpp"
#include "utils/geometry.hpp"

PYBIND11_MODULE(px4_ros2_py, m)
{
  m.doc() = R"pbdoc(
        Python bindings for px4_ros2_cpp.
    )pbdoc";

  bindNodeWrapper(m);

  {
    auto submodule = m.def_submodule("geometry");
    bindGeometry(submodule);
  }

  {
    auto submodule = m.def_submodule("components");
    bindMode(submodule);
    bindModeExecutor(submodule);
  }

  {
    auto submodule = m.def_submodule("control");
    bindSetpointMulticopterGoto(submodule);
  }
  {
    auto submodule = m.def_submodule("odometry");
    bindLocalPosition(submodule);
  }
}