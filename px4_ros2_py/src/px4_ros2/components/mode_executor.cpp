/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "mode_executor.hpp"

#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <px4_ros2/components/mode_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mode.hpp"

namespace py = pybind11;

// Python trampoline class for the virtual methods
class PyModeExecutorBase : public px4_ros2::ModeExecutorBase {
 public:
  using ModeExecutorBase::ModeExecutorBase;

  ~PyModeExecutorBase() override = default;

  void onActivate() override
  {
    PYBIND11_OVERRIDE_PURE_NAME(void, px4_ros2::ModeExecutorBase, "on_activate", onActivate, );
  }

  void onDeactivate(DeactivateReason reason) override
  {
    PYBIND11_OVERRIDE_PURE_NAME(void, px4_ros2::ModeExecutorBase, "on_deactivate", onDeactivate,
                                reason);
  }
};

void bindModeExecutor(py::module& m)
{
  // Bind the ModeExecutorBase::DeactivateReason enum
  py::enum_<px4_ros2::ModeExecutorBase::DeactivateReason>(m, "ModeExecutorDeactivateReason")
      .value("failsafe_activated", px4_ros2::ModeExecutorBase::DeactivateReason::FailsafeActivated)
      .value("other", px4_ros2::ModeExecutorBase::DeactivateReason::Other);

  // Bind the ModeExecutorBase class
  py::class_<px4_ros2::ModeExecutorBase, PyModeExecutorBase,
             std::shared_ptr<px4_ros2::ModeExecutorBase>>
      mode_executor_base(m, "ModeExecutorBase");

  mode_executor_base.def("register", &px4_ros2::ModeExecutorBase::doRegister)
      .def("on_activate", &px4_ros2::ModeExecutorBase::onActivate)
      .def("on_deactivate", &px4_ros2::ModeExecutorBase::onDeactivate)
      .def("schedule_mode",
           py::overload_cast<px4_ros2::ModeBase::ModeID,
                             const px4_ros2::ModeExecutorBase::CompletedCallback&, bool>(
               &px4_ros2::ModeExecutorBase::scheduleMode),
           py::arg("mode_id"), py::arg("on_completed"), py::arg("forced") = false)
      .def("takeoff", &px4_ros2::ModeExecutorBase::takeoff, py::arg("on_completed"),
           py::arg("altitude") = NAN, py::arg("heading") = NAN)
      .def("land", &px4_ros2::ModeExecutorBase::land)
      .def("rtl", &px4_ros2::ModeExecutorBase::rtl);

  // Constructor
  mode_executor_base.def(py::init([](px4_ros2::ModeBase& mode) {
                           return std::make_shared<PyModeExecutorBase>(
                               px4_ros2::ModeExecutorBase::Settings{}, mode);
                         }),
                         py::arg("mode"));
}
