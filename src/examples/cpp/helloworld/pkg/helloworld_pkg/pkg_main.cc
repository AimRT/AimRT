// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "helloworld_module/helloworld_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"HelloWorldModule", []() -> aimrt::ModuleBase* {
       return new aimrt::examples::cpp::helloworld::helloworld_module::HelloWorldModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
