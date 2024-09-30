// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace aimrt::examples::cpp::pb_rpc::normal_rpc_sync_server_module {

void SetLogger(aimrt::logger::LoggerRef);
aimrt::logger::LoggerRef GetLogger();

}  // namespace aimrt::examples::cpp::pb_rpc::normal_rpc_sync_server_module
