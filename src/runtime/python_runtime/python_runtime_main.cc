// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "pybind11/pybind11.h"

#include "python_runtime/export_channel.h"
#include "python_runtime/export_configurator.h"
#include "python_runtime/export_core.h"
#include "python_runtime/export_core_runtime.h"
#include "python_runtime/export_executor.h"
#include "python_runtime/export_logger.h"
#include "python_runtime/export_module_base.h"
#include "python_runtime/export_parameter.h"
#include "python_runtime/export_pb_type_support.h"
#include "python_runtime/export_rpc.h"

#ifdef AIMRT_BUILD_WITH_ROS2
  #include "python_runtime/export_ros2_type_support.h"
#endif

using namespace aimrt::runtime::python_runtime;

PYBIND11_MODULE(aimrt_python_runtime, m) {
  m.doc() = "AimRT Python Runtime Module";

  // core runtime
  ExportCoreOptions(m);
  ExportCore(m);

  // type support
  ExportPbTypeSupport(m);
#ifdef AIMRT_BUILD_WITH_ROS2
  ExportRos2TypeSupport(m);
#endif

  // core handle
  ExportModuleInfo(m);
  ExportCoreRef(m);

  // module base
  ExportModuleBase(m);

  // configure
  ExportConfiguratorRef(m);

  // log
  ExportLoggerRef(m);

  // executor
  ExportExecutorManagerRef(m);
  ExportExecutorRef(m);

  // channel
  ExportContext(m);
  ExportPublisherRef(m);
  ExportSubscriberRef(m);
  ExportChannelHandleRef(m);

  // rpc
  ExportRpcStatus(m);
  ExportRpcContext(m);
  ExportRpcServiceBase(m);
  ExportRpcHandleRef(m);
  ExportRpcProxyBase(m);

  // parameter
  ExportParameter(m);
}
