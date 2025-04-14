// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "benchmark_publisher_module/benchmark_publisher_module.h"
#include "normal_publisher_module/normal_publisher_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"NormalPublisherModule", []() -> aimrt::ModuleBase* {
       return new aimrt::examples::cpp::ros2_chn::normal_publisher_module::NormalPublisherModule();
     }},
    {"BenchmarkPublisherModule", []() -> aimrt::ModuleBase* {
       return new aimrt::examples::cpp::ros2_chn::benchmark_publisher_module::BenchmarkPublisherModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
