// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "pybind11/pybind11.h"

#include <utility>

#include "aimrt_module_cpp_interface/module_base.h"

namespace aimrt::runtime::python_runtime {

class PyModuleBaseAdapter : public ModuleBase {
 public:
  using ModuleBase::ModuleBase;

  ModuleInfo Info() const override {
    PYBIND11_OVERRIDE_PURE(ModuleInfo, ModuleBase, Info);
  }

  bool Initialize(CoreRef core) override {
    PYBIND11_OVERRIDE_PURE(bool, ModuleBase, Initialize, core);
  }

  bool Start() override {
    PYBIND11_OVERRIDE_PURE(bool, ModuleBase, Start);
  }

  void Shutdown() override {
    PYBIND11_OVERRIDE_PURE(void, ModuleBase, Shutdown);
  }
};

inline void ExportModuleBase(pybind11::object m) {
  pybind11::class_<ModuleBase, PyModuleBaseAdapter, std::shared_ptr<ModuleBase>>(std::move(m), "ModuleBase")
      .def(pybind11::init<>())
      .def("Info", &ModuleBase::Info)
      .def("Initialize", &ModuleBase::Initialize)
      .def("Start", &ModuleBase::Start)
      .def("Shutdown", &ModuleBase::Shutdown);
}

}  // namespace aimrt::runtime::python_runtime
