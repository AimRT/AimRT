// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <functional>
#include <string_view>
#include <tuple>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "chn_publisher_module/channel_publisher_module.h"
#include "chn_subscriber_on_exeutor_module/chn_subscriber_on_exeutor_module.h"
#include "chn_subscriber_on_module/channel_subscriber_module.h"
#include "executor/executor_module.h"

using aimrt::examples::cpp::context::channel_publisher_module::ChannelPublisherModule;
using aimrt::examples::cpp::context::ChnSubscriberInlineModule::ChnSubscriberInlineModule;
using aimrt::examples::cpp::context::ChnSubscriberOnExecutorModule::ChnSubscriberOnExecutorModule;
using aimrt::examples::cpp::context::executor_module::ExecutorModule;

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"ContextExecutorModule", []() -> aimrt::ModuleBase* { return new ExecutorModule(); }},
    {"ContextChannelPublisherModule", []() -> aimrt::ModuleBase* { return new ChannelPublisherModule(); }},
    {"ContextSubscriberOnExecutorModule", []() -> aimrt::ModuleBase* { return new ChnSubscriberOnExecutorModule(); }},
    {"ContextSubscriberInlineModule", []() -> aimrt::ModuleBase* { return new ChnSubscriberInlineModule(); }},
};

AIMRT_PKG_MAIN(aimrt_module_register_array)
