// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "aimrt_module_cpp_interface/channel/channel_context.h"
#include "util/stl_tool.h"

namespace aimrt::channel {

TEST(CHANNEL_CONTEXT_TEST, Context) {
  Context ctx;

  EXPECT_EQ(ctx.GetMetaKeys().size(), 0);
  EXPECT_EQ(ctx.GetMetaKeyVals().size(), 0);

  ctx.SetMetaValue("key1", "val1");
  ctx.SetMetaValue("key2", "val2");

  EXPECT_EQ(ctx.GetMetaKeys().size(), 2);
  EXPECT_EQ(ctx.GetMetaKeyVals().size(), 2);

  EXPECT_EQ(ctx.GetMetaValue("key1"), "val1");
  EXPECT_EQ(ctx.GetMetaValue("key2"), "val2");
  EXPECT_EQ(ctx.GetMetaValue("key3"), "");

  EXPECT_TRUE(aimrt::common::util::CheckContainerEqualNoOrder(
      ctx.GetMetaKeys(),
      std::vector<std::string_view>{"key1", "key2"}));
}

TEST(CHANNEL_CONTEXT_TEST, ContextRef) {
  Context real_ctx;
  ContextRef ctx(real_ctx);

  EXPECT_EQ(ctx.GetMetaKeys().size(), 0);
  EXPECT_EQ(ctx.GetMetaKeyVals().size(), 0);

  ctx.SetMetaValue("key1", "val1");
  ctx.SetMetaValue("key2", "val2");

  EXPECT_EQ(ctx.GetMetaKeys().size(), 2);
  EXPECT_EQ(ctx.GetMetaKeyVals().size(), 2);

  EXPECT_EQ(ctx.GetMetaValue("key1"), "val1");
  EXPECT_EQ(ctx.GetMetaValue("key2"), "val2");
  EXPECT_EQ(ctx.GetMetaValue("key3"), "");

  EXPECT_TRUE(aimrt::common::util::CheckContainerEqualNoOrder(
      ctx.GetMetaKeys(),
      std::vector<std::string_view>{"key1", "key2"}));
}

}  // namespace aimrt::channel
