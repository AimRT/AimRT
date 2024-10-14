// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <utility>

#include "aimrt_module_cpp_interface/core.h"

#include "pybind11/pybind11.h"

namespace aimrt::runtime::python_runtime {

inline void ExportModuleInfo(pybind11::object m) {
  pybind11::class_<ModuleInfo>(std::move(m), "ModuleInfo")
      .def(pybind11::init<>())
      .def_readwrite("name", &ModuleInfo::name)
      .def_readwrite("major_version", &ModuleInfo::major_version)
      .def_readwrite("minor_version", &ModuleInfo::minor_version)
      .def_readwrite("patch_version", &ModuleInfo::patch_version)
      .def_readwrite("build_version", &ModuleInfo::build_version)
      .def_readwrite("author", &ModuleInfo::author)
      .def_readwrite("description", &ModuleInfo::description);
}

inline void ExportCoreRef(pybind11::object m) {
  pybind11::class_<CoreRef>(std::move(m), "CoreRef")
      .def(pybind11::init<>())
      .def("__bool__", &CoreRef::operator bool)
      .def("Info", &CoreRef::Info)
      .def("GetConfigurator", &CoreRef::GetConfigurator)
      .def("GetLogger", &CoreRef::GetLogger)
      .def("GetExecutorManager", &CoreRef::GetExecutorManager)
      .def("GetRpcHandle", &CoreRef::GetRpcHandle)
      .def("GetChannelHandle", &CoreRef::GetChannelHandle)
      .def("GetParameterHandle", &CoreRef::GetParameterHandle);
}

}  // namespace aimrt::runtime::python_runtime