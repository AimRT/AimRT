// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "pybind11/pybind11.h"

#include <utility>

#include "aimrt_module_cpp_interface/configurator/configurator.h"

namespace aimrt::runtime::python_runtime {

inline void ExportConfiguratorRef(pybind11::object m) {
  using aimrt::configurator::ConfiguratorRef;

  pybind11::class_<ConfiguratorRef>(std::move(m), "ConfiguratorRef")
      .def(pybind11::init<>())
      .def("__bool__", &ConfiguratorRef::operator bool)
      .def("GetConfigFilePath", &ConfiguratorRef::GetConfigFilePath);
}

}  // namespace aimrt::runtime::python_runtime