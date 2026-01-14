/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "node_wrapper.hpp"

namespace py = pybind11;

NodeWrapper::NodeWrapper(const std::string& node_name, bool debug_output)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  _node = std::make_shared<rclcpp::Node>(node_name);
  if (debug_output) {
    auto ret = rcutils_logging_set_logger_level(_node->get_logger().get_name(),
                                                RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
      rcutils_reset_error();
    }
  }
  RCLCPP_DEBUG(_node->get_logger(), "Initialize rclcpp");
}

NodeWrapper::~NodeWrapper()
{
  rclcpp::shutdown();
  if (_thread.joinable()) {
    _thread.join();
  }
}

void NodeWrapper::spin()
{
  rclcpp::spin(_node);
}

void NodeWrapper::spinNonBlocking()
{
  _thread = std::thread([this] { rclcpp::spin(_node); });
}

void bindNodeWrapper(py::module& m)
{
  py::class_<NodeWrapper> node_wrapper(m, "Node");
  node_wrapper
      .def(py::init<const std::string&, bool>(), py::arg("node_name"),
           py::arg("debug_output") = false)
      .def("spin", &NodeWrapper::spin, "Execute the node (blocking)")
      .def("spin_non_blocking", &NodeWrapper::spinNonBlocking,
           "Execute the node (non-blocking in a background thread)");
}
