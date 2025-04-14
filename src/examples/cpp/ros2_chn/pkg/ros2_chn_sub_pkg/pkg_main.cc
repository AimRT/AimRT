// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "benchmark_subscriber_module/benchmark_subscriber_module.h"
#include "normal_subscriber_module/normal_subscriber_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"NormalSubscriberModule", []() -> aimrt::ModuleBase* {
       return new aimrt::examples::cpp::ros2_chn::normal_subscriber_module::NormalSubscriberModule();
     }},
    {"BenchmarkSubscriberModule", []() -> aimrt::ModuleBase* {
       return new aimrt::examples::cpp::ros2_chn::benchmark_subscriber_module::BenchmarkSubscriberModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
