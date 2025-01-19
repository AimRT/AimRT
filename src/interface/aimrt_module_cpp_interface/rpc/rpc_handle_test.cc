// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"

namespace aimrt::rpc {

TEST(RPC_HANDLE_TEST, GetFullFuncName) {
  EXPECT_EQ(GetFullFuncName("type", "service", "func"), "type:/service/func");
  EXPECT_EQ(GetFullFuncName("", "", ""), "://");
}

TEST(RPC_HANDLE_TEST, GetFuncNameWithoutPrefix) {
  EXPECT_EQ(GetFuncNameWithoutPrefix("test"), "test");
  EXPECT_EQ(GetFuncNameWithoutPrefix(":/"), "/");
  EXPECT_EQ(GetFuncNameWithoutPrefix("type:/service/func"), "/service/func");
}

}  // namespace aimrt::rpc
