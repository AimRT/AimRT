// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#include <pybind11/pybind11.h>

#include "python_runtime/export_ros2_channel.h"
#include "python_runtime/export_ros2_rpc.h"
#include "python_runtime/export_ros2_type_support.h"

using namespace aimrt::runtime::python_runtime;

PYBIND11_MODULE(aimrt_python_runtime_ros2, m) {
  m.doc() = "AimRT Python support for ROS2 interface";

  // type support
  ExportRos2TypeSupport(m);

  // channel
  ExportRos2PublisherFunc(m);
  ExportRos2SubscribeFunc(m);

  // rpc
  ExportRos2RpcServiceFunc(m);
  ExportRos2ClientFunc(m);
}
