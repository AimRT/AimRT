// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#pragma once

#include "pybind11/pybind11.h"

#include "aimrt_module_cpp_interface/util/version.h"

namespace aimrt::runtime::python_runtime {

inline void ExportVersion(pybind11::object m) {
  m.attr("AimRT_RUNTIME_VERSION_INT") = AIMRT_RUNTIME_VERSION_INT;
  // This needs to be updated when the code generator is updated
  // and forward compability is broken.
  m.attr("AIMRT_MIN_GENCODE_VERSION_INT") = 10000;
}

}  // namespace aimrt::runtime::python_runtime
