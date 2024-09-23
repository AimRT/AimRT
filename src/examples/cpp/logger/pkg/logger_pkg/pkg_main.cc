// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "logger_bench_module/logger_bench_module.h"
#include "logger_module/logger_module.h"

using namespace aimrt::examples::cpp::logger;

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"LoggerModule", []() -> aimrt::ModuleBase* { return new logger_module::LoggerModule(); }},
    {"LoggerBenchModule", []() -> aimrt::ModuleBase* { return new logger_bench_module::LoggerBenchModule(); }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
