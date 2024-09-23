// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <utility>

#include "aimrt_module_cpp_interface/module_base.h"
#include "core/aimrt_core.h"

#include "pybind11/pybind11.h"

namespace aimrt::runtime::python_runtime {

inline void ExportCoreOptions(pybind11::object m) {
  using namespace aimrt::runtime::core;

  pybind11::class_<AimRTCore::Options>(std::move(m), "CoreOptions")
      .def(pybind11::init<>())
      .def_readwrite("cfg_file_path", &AimRTCore::Options::cfg_file_path);
}

inline void PyCoreStart(aimrt::runtime::core::AimRTCore& core) {
  // 阻塞之前需要释放gil锁
  pybind11::gil_scoped_release release;
  core.Start();
  pybind11::gil_scoped_acquire acquire;
}

inline void PyCoreRegisterModule(
    aimrt::runtime::core::AimRTCore& core, const aimrt::ModuleBase* module) {
  AIMRT_HL_CHECK_ERROR_THROW(core.GetLogger(), module, "module is null!");
  core.GetModuleManager().RegisterModule("core-py", module->NativeHandle());
}

inline aimrt::CoreRef PyCoreCreateModule(
    aimrt::runtime::core::AimRTCore& core, std::string_view module_name) {
  return aimrt::CoreRef(core.GetModuleManager().CreateModule("core-py", module_name));
}

inline void ExportCore(pybind11::object m) {
  using namespace aimrt::runtime::core;

  pybind11::class_<AimRTCore>(std::move(m), "Core")
      .def(pybind11::init<>())
      .def("Initialize", &AimRTCore::Initialize)
      .def("Start", &PyCoreStart)
      .def("Shutdown", &AimRTCore::Shutdown)
      .def("RegisterModule", &PyCoreRegisterModule)
      .def("CreateModule", &PyCoreCreateModule)
      .def("GenInitializationReport", &AimRTCore::GenInitializationReport);
}

}  // namespace aimrt::runtime::python_runtime
