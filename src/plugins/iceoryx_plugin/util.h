// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include <iomanip>
#include "core/channel/channel_backend_tools.h"
#include "core/channel/channel_msg_wrapper.h"
#include "iceoryx_plugin/global.h"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

#if defined(_WIN32)
  #include <windows.h>
#else
  #include <unistd.h>
#endif

namespace aimrt::plugins::iceoryx_plugin {

using IdString_t = iox::capro::IdString_t;

constexpr uint64_t kIoxShmInitSize = 1024;  // default vaule of shm_init_size for iceoryx

iox::capro::ServiceDescription Url2ServiceDescription(std::string& url);

std::string GetPid();

using namespace aimrt::runtime::core::channel;
std::pair<std::shared_ptr<aimrt::util::BufferArrayView>, size_t> SerializeMsgSupportedIceoryx(
    MsgWrapper& msg_wrapper, std::string_view serialization_type, aimrt::util::BufferArrayAllocatorRef allocator);

}  // namespace aimrt::plugins::iceoryx_plugin