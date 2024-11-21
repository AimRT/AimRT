// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "pybind11/pybind11.h"

#include <utility>

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace aimrt::runtime::python_runtime {

inline void ExportLoggerRef(pybind11::object m) {
  using aimrt::logger::LoggerRef;

  pybind11::class_<LoggerRef>(std::move(m), "LoggerRef")
      .def(pybind11::init<>())
      .def("__bool__", &LoggerRef::operator bool)
      .def("GetLogLevel", &LoggerRef::GetLogLevel)
      .def("Log", &LoggerRef::Log,
           pybind11::arg("lvl"),
           pybind11::arg("line"),
           pybind11::arg("column"),
           pybind11::arg("file_name"),
           pybind11::arg("function_name"),
           pybind11::arg("log_data"),
           pybind11::arg("log_data_size"));
}

}  // namespace aimrt::runtime::python_runtime