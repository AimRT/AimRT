// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "aimrt_module_cpp_interface/rpc/rpc_status.h"

namespace aimrt::rpc {

TEST(RPC_STATUS_TEST, Status_base) {
  Status s;

  EXPECT_EQ(s.Code(), AIMRT_RPC_STATUS_OK);
  EXPECT_TRUE(s);
  EXPECT_STREQ(s.ToString().c_str(), "suc, code 0, msg: OK");
}

TEST(RPC_STATUS_TEST, Status_error) {
  Status s(AIMRT_RPC_STATUS_UNKNOWN);

  EXPECT_EQ(s.Code(), AIMRT_RPC_STATUS_UNKNOWN);
  EXPECT_FALSE(s);
  EXPECT_STREQ(s.ToString().c_str(), "fail, code 1, msg: Unknown error");
}

}  // namespace aimrt::rpc
