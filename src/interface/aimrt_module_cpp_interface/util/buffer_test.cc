// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "aimrt_module_cpp_interface/util/buffer.h"
#include "aimrt_module_cpp_interface/util/simple_buffer_array_allocator.h"

namespace aimrt::util {

bool CheckBufferEqual(const aimrt_buffer_t& lhs, const aimrt_buffer_t& rhs) {
  return ((lhs.data == rhs.data) && (lhs.len == rhs.len));
}

TEST(BUFFER_TEST, Base) {
  // init
  BufferArray buffer_array(SimpleBufferArrayAllocator::NativeHandle());
  EXPECT_EQ(buffer_array.Data(), nullptr);
  EXPECT_EQ(buffer_array.Size(), 0);
  EXPECT_EQ(buffer_array.Capacity(), 0);

  // first buffer
  aimrt_buffer_t buffer = buffer_array.NewBuffer(128);
  EXPECT_EQ(buffer.len, 128);
  EXPECT_EQ(buffer_array.Size(), 1);
  EXPECT_EQ(buffer_array.Capacity(), 2);

  aimrt_buffer_t* buffer_array_data = buffer_array.Data();
  EXPECT_TRUE(CheckBufferEqual(buffer_array_data[0], buffer));

  // second buffer
  aimrt_buffer_t buffer2 = buffer_array.NewBuffer(64);
  EXPECT_EQ(buffer2.len, 64);
  EXPECT_EQ(buffer_array.Size(), 2);
  EXPECT_EQ(buffer_array.Capacity(), 2);

  buffer_array_data = buffer_array.Data();
  EXPECT_TRUE(CheckBufferEqual(buffer_array_data[0], buffer));
  EXPECT_TRUE(CheckBufferEqual(buffer_array_data[1], buffer2));

  // third buffer
  aimrt_buffer_t buffer3 = buffer_array.NewBuffer(512);
  EXPECT_EQ(buffer3.len, 512);
  EXPECT_EQ(buffer_array.Size(), 3);
  EXPECT_EQ(buffer_array.Capacity(), 4);

  buffer_array_data = buffer_array.Data();
  EXPECT_TRUE(CheckBufferEqual(buffer_array_data[0], buffer));
  EXPECT_TRUE(CheckBufferEqual(buffer_array_data[1], buffer2));
  EXPECT_TRUE(CheckBufferEqual(buffer_array_data[2], buffer3));

  // buffer array view
  BufferArrayView buffer_array_view(buffer_array);
  EXPECT_EQ(buffer_array_view.Size(), 3);

  EXPECT_EQ(buffer_array_view.BufferSize(), buffer_array.BufferSize());
}

}  // namespace aimrt::util
