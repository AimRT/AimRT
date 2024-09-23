// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "normal_publisher_module/normal_publisher_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"NormalPublisherModule", []() -> aimrt::ModuleBase* {
       return new aimrt::examples::cpp::ros2_channel::normal_publisher_module::NormalPublisherModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
