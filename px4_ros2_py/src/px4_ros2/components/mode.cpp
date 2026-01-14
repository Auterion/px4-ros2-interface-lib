/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "mode.hpp"

#include <px4_ros2/components/mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "node_wrapper.hpp"

namespace py = pybind11;

// Python trampoline class for the virtual methods
class PyModeBase : public px4_ros2::ModeBase {
 public:
  using ModeBase::ModeBase;

  ~PyModeBase() override = default;

  void onActivate() override
  {
    PYBIND11_OVERRIDE_NAME(void, px4_ros2::ModeBase, "on_activate", onActivate, );
  }

  void onDeactivate() override
  {
    PYBIND11_OVERRIDE_NAME(void, px4_ros2::ModeBase, "on_deactivate", onDeactivate, );
  }

  void updateSetpoint(float dt_s) override
  {
    PYBIND11_OVERRIDE_PURE_NAME(void, px4_ros2::ModeBase, "update_setpoint", updateSetpoint, dt_s);
  }
};

void bindMode(py::module& m)
{
  // Bind the Result enum
  py::enum_<px4_ros2::Result> result_enum(m, "Result");
  result_enum.value("deactivated", px4_ros2::Result::Deactivated)
      .value("interrupted", px4_ros2::Result::Interrupted)
      .value("mode_failure_other", px4_ros2::Result::ModeFailureOther)
      .value("rejected", px4_ros2::Result::Rejected)
      .value("success", px4_ros2::Result::Success)
      .value("timeout", px4_ros2::Result::Timeout);

  // Register the Context class (required for the setpoints)
  py::class_<px4_ros2::Context, std::shared_ptr<px4_ros2::Context>> context(m, "Context");

  // Bind the ModeBase class
  py::class_<px4_ros2::ModeBase, PyModeBase, px4_ros2::Context, std::shared_ptr<px4_ros2::ModeBase>>
      mode_base(m, "ModeBase");

  mode_base.def("register", &px4_ros2::ModeBase::doRegister)
      .def("on_activate", &px4_ros2::ModeBase::onActivate)
      .def("on_deactivate", &px4_ros2::ModeBase::onDeactivate)
      .def("update_setpoint", &px4_ros2::ModeBase::updateSetpoint)
      .def("id", &px4_ros2::ModeBase::id)
      .def("completed", &px4_ros2::ModeBase::completed,
           py::arg("result") = px4_ros2::Result::Success);

  // Constructor
  mode_base.def(
      py::init([](NodeWrapper& node, const std::string& mode_name,
                  bool activate_even_while_disarmed = false,
                  int replace_internal_mode = px4_ros2::ModeBase::kModeIDInvalid,
                  bool prevent_arming = false, bool user_selectable = true) {
        px4_ros2::ModeBase::Settings settings(mode_name);
        settings.activate_even_while_disarmed = activate_even_while_disarmed;
        settings.replace_internal_mode =
            static_cast<px4_ros2::ModeBase::ModeID>(replace_internal_mode);
        settings.prevent_arming = prevent_arming;
        settings.user_selectable = user_selectable;
        return std::make_shared<PyModeBase>(*node.node(), settings);
      }),
      py::arg("node"), py::arg("mode_name"), py::arg("activate_even_while_disarmed") = false,
      py::arg("replace_internal_mode") = static_cast<int>(px4_ros2::ModeBase::kModeIDInvalid),
      py::arg("prevent_arming") = false, py::arg("user_selectable") = true);
}
