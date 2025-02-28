// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/buffer_util.h"

namespace aimrt::common::util {

TEST(SetGetBufTest, base) {
  char buf[8];

  const uint16_t n16 = 0x0123;
  SetBufFromUint16(buf, n16);
  EXPECT_EQ(GetUint16FromBuf(buf), n16);

  const uint32_t n32 = 0x01234567;
  SetBufFromUint32(buf, n32);
  EXPECT_EQ(GetUint32FromBuf(buf), n32);

  const uint64_t n64 = 0x0123456789ABCDEF;
  SetBufFromUint64(buf, n64);
  EXPECT_EQ(GetUint64FromBuf(buf), n64);
}

TEST(BufferOperatorTest, base) {
  char buf[1024];

  uint8_t n8 = 0x01;
  uint16_t n16 = 0x0123;
  uint32_t n32 = 0x01234567;
  uint64_t n64 = 0x0123456789ABCDEF;
  std::string_view s("0123456789ABCDEF");

  BufferOperator op(buf, sizeof(buf));

  op.SetUint8(n8);
  op.SetString(s, BufferLenType::kUInt8);

  op.JumpTo(100);

  op.SetUint16(n16);
  op.SetString(s, BufferLenType::kUInt16);

  op.Skip(100);

  op.SetUint32(n32);
  op.SetString(s, BufferLenType::kUInt32);
  op.SetUint64(n64);
  op.SetString(s, BufferLenType::kUInt64);

  op.SetBuffer(std::span<const char>(s.data(), s.size()));

  ConstBufferOperator cop(buf, sizeof(buf));

  ASSERT_EQ(cop.GetUint8(), n8);
  ASSERT_EQ(cop.GetString(BufferLenType::kUInt8), s);

  cop.JumpTo(100);

  ASSERT_EQ(cop.GetUint16(), n16);
  ASSERT_EQ(cop.GetString(BufferLenType::kUInt16), s);

  cop.Skip(100);

  ASSERT_EQ(cop.GetUint32(), n32);
  ASSERT_EQ(cop.GetString(BufferLenType::kUInt32), s);
  ASSERT_EQ(cop.GetUint64(), n64);
  ASSERT_EQ(cop.GetString(BufferLenType::kUInt64), s);

  auto buf_span = cop.GetBuffer(s.size());
  ASSERT_EQ(std::string_view(buf_span.data(), buf_span.size()), s);

  ASSERT_EQ(op.GetRemainingSize(), cop.GetRemainingSize());
}

}  // namespace aimrt::common::util