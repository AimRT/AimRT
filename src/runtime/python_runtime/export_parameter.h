// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/parameter/parameter_handle.h"

#include "pybind11/pybind11.h"

namespace aimrt::runtime::python_runtime {

inline void ExportParameter(pybind11::object m) {
  using aimrt::parameter::ParameterHandleRef;

  pybind11::class_<ParameterHandleRef>(std::move(m), "ParameterHandleRef")
      .def(pybind11::init<>())
      .def("__bool__", &ParameterHandleRef::operator bool)
      .def("GetParameter", &ParameterHandleRef::GetParameter)
      .def("SetParameter", &ParameterHandleRef::SetParameter);
}

}  // namespace aimrt::runtime::python_runtime