// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "pybind11/pybind11.h"

#include <utility>

#include "aimrt_module_cpp_interface/executor/executor_manager.h"

#include "pybind11/chrono.h"
#include "pybind11/functional.h"
#include "pybind11/stl.h"

namespace aimrt::runtime::python_runtime {

inline void ExportExecutorManagerRef(pybind11::object m) {
  using aimrt::executor::ExecutorManagerRef;

  pybind11::class_<ExecutorManagerRef>(std::move(m), "ExecutorManagerRef")
      .def(pybind11::init<>())
      .def("__bool__", &ExecutorManagerRef::operator bool)
      .def("GetExecutor", &ExecutorManagerRef::GetExecutor);
}

inline void PyExecutorRefExecuteWrapper(
    aimrt::executor::ExecutorRef& executor, std::function<void()>&& task) {
  executor.Execute(std::move(task));
}

inline void PyExecutorRefExecuteAtWrapper(
    aimrt::executor::ExecutorRef& executor,
    std::chrono::system_clock::time_point tp,
    std::function<void()>&& task) {
  executor.ExecuteAt(tp, std::move(task));
}

inline void PyExecutorRefExecuteAfterWrapper(
    aimrt::executor::ExecutorRef& executor,
    std::chrono::nanoseconds dt,
    std::function<void()>&& task) {
  executor.ExecuteAfter(dt, std::move(task));
}

inline void ExportExecutorRef(pybind11::object m) {
  using aimrt::executor::ExecutorRef;

  pybind11::class_<ExecutorRef>(std::move(m), "ExecutorRef")
      .def(pybind11::init<>())
      .def("__bool__", &ExecutorRef::operator bool)
      .def("Type", &ExecutorRef::Type)
      .def("Name", &ExecutorRef::Name)
      .def("ThreadSafe", &ExecutorRef::ThreadSafe)
      .def("IsInCurrentExecutor", &ExecutorRef::IsInCurrentExecutor)
      .def("SupportTimerSchedule", &ExecutorRef::SupportTimerSchedule)
      .def("Execute", &PyExecutorRefExecuteWrapper)
      .def("Now", &ExecutorRef::Now)
      .def("ExecuteAt", &PyExecutorRefExecuteAtWrapper)
      .def("ExecuteAfter", &PyExecutorRefExecuteAfterWrapper);
}

}  // namespace aimrt::runtime::python_runtime
