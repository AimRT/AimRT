// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "normal_publisher_module/normal_publisher_module.h"
#include "normal_subscriber_module/normal_subscriber_module.h"

using namespace aimrt::examples::cpp::ros2_channel;

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>> aimrt_module_register_array[]{
    {"NormalPublisherModule", []() -> aimrt::ModuleBase* {
       return new normal_publisher_module::NormalPublisherModule();
     }},
    {"NormalSubscriberModule", []() -> aimrt::ModuleBase* {
       return new normal_subscriber_module::NormalSubscriberModule();
     }}};

AIMRT_PKG_MAIN(aimrt_module_register_array)
