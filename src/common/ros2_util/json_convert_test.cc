// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include <cmath>

#include "ros2_util/json_convert.h"

#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/byte.hpp"
#include "example_interfaces/msg/char.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/int16.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/string.hpp"
#include "example_interfaces/msg/u_int16.hpp"
#include "example_interfaces/msg/u_int32.hpp"
#include "example_interfaces/msg/u_int64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "example_interfaces/msg/w_string.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/bounded_sequences.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/nested.hpp"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"
#include "test_msgs/msg/w_strings.hpp"

namespace aimrt::common::ros2_util {

static std::vector<test_msgs::msg::BasicTypes::SharedPtr>
GetMessagesBasicTypes() {
  std::vector<test_msgs::msg::BasicTypes::SharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::BasicTypes>();
    msg->bool_value = false;
    msg->byte_value = 0;
    msg->char_value = 0;
    msg->float32_value = 0.0f;
    msg->float64_value = 0;
    msg->int8_value = 0;
    msg->uint8_value = 0;
    msg->int16_value = 0;
    msg->uint16_value = 0;
    msg->int32_value = 0;
    msg->uint32_value = 0;
    msg->int64_value = 0;
    msg->uint64_value = 0;
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::BasicTypes>();
    msg->bool_value = true;
    msg->byte_value = 255;
    msg->char_value = 255;
    msg->float32_value = 1.125f;
    msg->float64_value = 1.125;
    msg->int8_value = (std::numeric_limits<int8_t>::max)();
    msg->uint8_value = (std::numeric_limits<uint8_t>::max)();
    msg->int16_value = (std::numeric_limits<int16_t>::max)();
    msg->uint16_value = (std::numeric_limits<uint16_t>::max)();
    msg->int32_value = (std::numeric_limits<int32_t>::max)();
    msg->uint32_value = (std::numeric_limits<uint32_t>::max)();
    msg->int64_value = (std::numeric_limits<int64_t>::max)();
    msg->uint64_value = (std::numeric_limits<uint64_t>::max)();
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::BasicTypes>();
    msg->bool_value = false;
    msg->byte_value = 0;
    msg->char_value = 0;
    msg->float32_value = -2.125f;
    msg->float64_value = -2.125;
    msg->int8_value = (std::numeric_limits<int8_t>::min)();
    msg->uint8_value = 0;
    msg->int16_value = (std::numeric_limits<int16_t>::min)();
    msg->uint16_value = 0;
    msg->int32_value = (std::numeric_limits<int32_t>::min)();
    msg->uint32_value = 0;
    msg->int64_value = (std::numeric_limits<int64_t>::min)();
    msg->uint64_value = 0;
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::BasicTypes>();
    msg->bool_value = true;
    msg->byte_value = 1;
    msg->char_value = 1;
    msg->float32_value = 1.0f;
    msg->float64_value = 1;
    msg->int8_value = 1;
    msg->uint8_value = 1;
    msg->int16_value = 1;
    msg->uint16_value = 1;
    msg->int32_value = 1;
    msg->uint32_value = 1;
    msg->int64_value = 1;
    msg->uint64_value = 1;
    messages.push_back(msg);
  }
  return messages;
}

static std::vector<test_msgs::msg::Arrays::SharedPtr>
GetMessagesArrays() {
  auto basic_types_msgs = GetMessagesBasicTypes();
  std::vector<test_msgs::msg::Arrays::SharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::Arrays>();
    msg->bool_values = {{false, true, false}};
    msg->byte_values = {{0, 0xff, 0}};
    msg->char_values = {{0, 255, 0}};
    msg->float32_values = {{0.0f, 1.125f, -2.125f}};
    msg->float64_values = {{0, 1.125, -2.125}};
    msg->int8_values = {{0, (std::numeric_limits<int8_t>::max)(), (std::numeric_limits<int8_t>::min)()}};
    msg->uint8_values = {{0, (std::numeric_limits<uint8_t>::max)(), 0}};
    msg->int16_values = {{0, (std::numeric_limits<int16_t>::max)(), (std::numeric_limits<int16_t>::min)()}};
    msg->uint16_values = {{0, (std::numeric_limits<uint16_t>::max)(), 0}};
    msg->int32_values = {{static_cast<int32_t>(0),
                          (std::numeric_limits<int32_t>::max)(),
                          (std::numeric_limits<int32_t>::min)()}};
    msg->uint32_values = {{0, (std::numeric_limits<uint32_t>::max)(), 0}};
    msg->int64_values[0] = 0;
    msg->int64_values[1] = (std::numeric_limits<int64_t>::max)();
    msg->int64_values[2] = (std::numeric_limits<int64_t>::min)();
    msg->uint64_values = {{0, (std::numeric_limits<uint64_t>::max)(), 0}};
    msg->string_values = {{"", "max value", "min value"}};
    msg->basic_types_values[0] = *basic_types_msgs[0];
    msg->basic_types_values[1] = *basic_types_msgs[1];
    msg->basic_types_values[2] = *basic_types_msgs[2];
    messages.push_back(msg);
  }
  return messages;
}

static std::vector<test_msgs::msg::BoundedSequences::SharedPtr>
GetMessagesBoundedSequences() {
  auto basic_types_msgs = GetMessagesBasicTypes();
  std::vector<test_msgs::msg::BoundedSequences::SharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::BoundedSequences>();
    msg->bool_values = {{false, true, false}};
    msg->byte_values = {{0, 1, 0xff}};
    msg->char_values = {{0, 1, 255}};
    msg->float32_values = {{0.0f, 1.125f, -2.125f}};
    msg->float64_values = {{0, 1.125, -2.125}};
    msg->int8_values = {{0, (std::numeric_limits<int8_t>::max)(), (std::numeric_limits<int8_t>::min)()}};
    msg->uint8_values = {{0, 1, (std::numeric_limits<uint8_t>::max)()}};
    msg->int16_values = {{0, (std::numeric_limits<int16_t>::max)(), (std::numeric_limits<int16_t>::min)()}};
    msg->uint16_values = {{0, 1, (std::numeric_limits<uint16_t>::max)()}};
    // The narrowing static cast is required to avoid build errors on Windows.
    msg->int32_values = {{static_cast<int32_t>(0),
                          (std::numeric_limits<int32_t>::max)(),
                          (std::numeric_limits<int32_t>::min)()}};
    msg->uint32_values = {{0, 1, (std::numeric_limits<uint32_t>::max)()}};
    msg->int64_values.resize(3);
    msg->int64_values[0] = 0;
    msg->int64_values[1] = (std::numeric_limits<int64_t>::max)();
    msg->int64_values[2] = (std::numeric_limits<int64_t>::min)();
    msg->uint64_values = {{0, 1, (std::numeric_limits<uint64_t>::max)()}};
    msg->string_values = {{"", "max value", "optional min value"}};
    msg->basic_types_values.resize(3);
    msg->basic_types_values[0] = *basic_types_msgs[0];
    msg->basic_types_values[1] = *basic_types_msgs[1];
    msg->basic_types_values[2] = *basic_types_msgs[2];
    msg->alignment_check = 2;
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::BoundedSequences>();
    // check default sequences
    msg->alignment_check = 4;
    messages.push_back(msg);
  }
  return messages;
}

static std::vector<test_msgs::msg::UnboundedSequences::SharedPtr>
GetMessagesUnboundedSequences() {
  auto basic_types_msgs = GetMessagesBasicTypes();
  std::vector<test_msgs::msg::UnboundedSequences::SharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::UnboundedSequences>();
    msg->bool_values = {};
    msg->byte_values = {};
    msg->char_values = {};
    msg->float32_values = {};
    msg->float64_values = {};
    msg->int8_values = {};
    msg->uint8_values = {};
    msg->int16_values = {};
    msg->uint16_values = {};
    msg->int32_values = {};
    msg->uint32_values = {};
    msg->int64_values = {};
    msg->uint64_values = {};
    msg->string_values = {};
    msg->basic_types_values = {};
    msg->alignment_check = 0;
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::UnboundedSequences>();
    msg->bool_values = {true};
    msg->byte_values = {0xff};
    msg->char_values = {255};
    msg->float32_values = {1.125f};
    msg->float64_values = {1.125};
    msg->int8_values = {(std::numeric_limits<int8_t>::max)()};
    msg->uint8_values = {(std::numeric_limits<uint8_t>::max)()};
    msg->int16_values = {(std::numeric_limits<int16_t>::max)()};
    msg->uint16_values = {(std::numeric_limits<uint16_t>::max)()};
    msg->int32_values = {(std::numeric_limits<int32_t>::max)()};
    msg->uint32_values = {(std::numeric_limits<uint32_t>::max)()};
    msg->int64_values = {(std::numeric_limits<int64_t>::max)()};
    msg->uint64_values = {(std::numeric_limits<uint64_t>::max)()};
    msg->string_values = {{"max value"}};
    msg->basic_types_values = {{*basic_types_msgs[0]}};
    msg->alignment_check = 1;
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::UnboundedSequences>();
    msg->bool_values = {{false, true}};
    msg->byte_values = {{0, 0xff}};
    msg->char_values = {{0, 255}};
    msg->float32_values = {{0.0f, 1.125f, -2.125f}};
    msg->float64_values = {{0, 1.125, -2.125}};
    msg->int8_values = {{0, (std::numeric_limits<int8_t>::max)(), (std::numeric_limits<int8_t>::min)()}};
    msg->uint8_values = {{0, (std::numeric_limits<uint8_t>::max)()}};
    msg->int16_values = {{0, (std::numeric_limits<int16_t>::max)(), (std::numeric_limits<int16_t>::min)()}};
    msg->uint16_values = {{0, (std::numeric_limits<uint16_t>::max)()}};
    // The narrowing static cast is required to avoid build errors on Windows.
    msg->int32_values = {{static_cast<int32_t>(0),
                          (std::numeric_limits<int32_t>::max)(),
                          (std::numeric_limits<int32_t>::min)()}};
    msg->uint32_values = {{0, (std::numeric_limits<uint32_t>::max)()}};
    msg->int64_values.resize(3);
    msg->int64_values[0] = 0;
    msg->int64_values[1] = (std::numeric_limits<int64_t>::max)();
    msg->int64_values[2] = (std::numeric_limits<int64_t>::min)();
    msg->uint64_values = {{0, (std::numeric_limits<uint64_t>::max)()}};
    msg->string_values = {{"", "max value", "optional min value"}};
    msg->basic_types_values.resize(3);
    msg->basic_types_values[0] = *basic_types_msgs[0];
    msg->basic_types_values[1] = *basic_types_msgs[1];
    msg->basic_types_values[2] = *basic_types_msgs[2];
    msg->alignment_check = 2;
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::UnboundedSequences>();
    // check sequences with more then 100 elements
    const size_t size = 1000;
    msg->bool_values.resize(size);
    msg->byte_values.resize(size);
    msg->char_values.resize(size);
    msg->float32_values.resize(size);
    msg->float64_values.resize(size);
    msg->int8_values.resize(size);
    msg->uint8_values.resize(size);
    msg->int16_values.resize(size);
    msg->uint16_values.resize(size);
    msg->int32_values.resize(size);
    msg->uint32_values.resize(size);
    msg->int64_values.resize(size);
    msg->uint64_values.resize(size);
    msg->string_values.resize(size);
    msg->basic_types_values.resize(size);
    for (size_t i = 0; i < size; ++i) {
      msg->bool_values[i] = (i % 2 != 0) ? true : false;
      msg->byte_values[i] = static_cast<uint8_t>(i);
      msg->char_values[i] = static_cast<uint8_t>(i);
      msg->float32_values[i] = 1.125f * i;
      msg->float64_values[i] = 1.125 * i;
      msg->int8_values[i] = static_cast<int8_t>(i);
      msg->uint8_values[i] = static_cast<uint8_t>(i);
      msg->int16_values[i] = static_cast<int16_t>(i);
      msg->uint16_values[i] = static_cast<uint16_t>(i);
      msg->int32_values[i] = static_cast<int32_t>(i);
      msg->uint32_values[i] = static_cast<uint32_t>(i);
      msg->int64_values[i] = i;
      msg->uint64_values[i] = i;
      msg->string_values[i] = std::to_string(i);
      msg->basic_types_values[i] = *basic_types_msgs[i % basic_types_msgs.size()];
    }
    msg->alignment_check = 3;
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::UnboundedSequences>();
    // check default sequences
    msg->alignment_check = 4;
    messages.push_back(msg);
  }
  return messages;
}

static std::vector<test_msgs::msg::MultiNested::SharedPtr>
GetMessagesMultiNested() {
  auto arrays_msgs = GetMessagesArrays();
  auto bounded_sequences_msgs = GetMessagesBoundedSequences();
  auto unbounded_sequences_msgs = GetMessagesUnboundedSequences();
  const std::size_t num_arrays = arrays_msgs.size();
  const std::size_t num_bounded_sequences = bounded_sequences_msgs.size();
  const std::size_t num_unbounded_sequences = unbounded_sequences_msgs.size();
  std::vector<test_msgs::msg::MultiNested::SharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::MultiNested>();
    for (std::size_t i = 0u; i < msg->array_of_arrays.size(); ++i) {
      msg->array_of_arrays[i] = *arrays_msgs[i % num_arrays];
    }
    for (std::size_t i = 0u; i < msg->array_of_bounded_sequences.size(); ++i) {
      msg->array_of_bounded_sequences[i] = *bounded_sequences_msgs[i % num_bounded_sequences];
    }
    for (std::size_t i = 0u; i < msg->array_of_unbounded_sequences.size(); ++i) {
      msg->array_of_unbounded_sequences[i] = *unbounded_sequences_msgs[i % num_unbounded_sequences];
    }
    const std::size_t sequence_size = 3u;
    msg->bounded_sequence_of_arrays.resize(sequence_size);
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->bounded_sequence_of_arrays[i] = *arrays_msgs[i % num_arrays];
    }
    msg->bounded_sequence_of_bounded_sequences.resize(sequence_size);
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->bounded_sequence_of_bounded_sequences[i] =
          *bounded_sequences_msgs[i % num_bounded_sequences];
    }
    msg->bounded_sequence_of_unbounded_sequences.resize(sequence_size);
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->bounded_sequence_of_unbounded_sequences[i] =
          *unbounded_sequences_msgs[i % num_unbounded_sequences];
    }
    msg->unbounded_sequence_of_arrays.resize(sequence_size);
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->unbounded_sequence_of_arrays[i] = *arrays_msgs[i % num_arrays];
    }
    msg->unbounded_sequence_of_bounded_sequences.resize(sequence_size);
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->unbounded_sequence_of_bounded_sequences[i] =
          *bounded_sequences_msgs[i % num_bounded_sequences];
    }
    msg->unbounded_sequence_of_unbounded_sequences.resize(sequence_size);
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->unbounded_sequence_of_unbounded_sequences[i] =
          *unbounded_sequences_msgs[i % num_unbounded_sequences];
    }
    messages.push_back(msg);
  }
  return messages;
}

static std::vector<test_msgs::msg::Nested::SharedPtr>
GetMessagesNested() {
  std::vector<test_msgs::msg::Nested::SharedPtr> messages;
  auto basic_types_msgs = GetMessagesBasicTypes();
  for (auto basic_types_msg : basic_types_msgs) {
    auto msg = std::make_shared<test_msgs::msg::Nested>();
    msg->basic_types_value = *basic_types_msg;
    messages.push_back(msg);
  }
  return messages;
}

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
AlmostEqual(T x, T y, int ulp) {
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp ||
         std::fabs(x - y) < std::numeric_limits<T>::min();
}

template <class RosType>
bool TestJsonToRos2Message(
    const std::string& json_str,
    const RosType& check_message,
    std::function<bool(const RosType&, const RosType&)> check_func =
        [](const RosType& lhs, const RosType& rhs) -> bool { return lhs == rhs; }) {
  RosType message;
  bool ret = JsonToMessage(json_str, GetIntrospectionTypeSupport<RosType>(), &message);
  if (!ret) return false;
  return check_func(message, check_message);
}

TEST(JsonToRos2Message, BasicType_Bool) {
  using RosType = ::example_interfaces::msg::Bool;

  {
    std::string json_str = "{\"data\":true}";
    RosType check_message;
    check_message.data = true;
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":false}";
    RosType check_message;
    check_message.data = false;
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_Float32) {
  using RosType = ::example_interfaces::msg::Float32;

  auto check_func = [](const RosType& lhs, const RosType& rhs) -> bool {
    return AlmostEqual(lhs.data, rhs.data, 2);
  };

  {
    std::string json_str = "{\"data\":42.42}";
    RosType check_message;
    check_message.data = 42.42f;
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message, check_func));
  }
}

TEST(JsonToRos2Message, BasicType_Float64) {
  using RosType = ::example_interfaces::msg::Float64;

  auto check_func = [](const RosType& lhs, const RosType& rhs) -> bool {
    return AlmostEqual(lhs.data, rhs.data, 2);
  };

  {
    std::string json_str = "{\"data\":42424242424242.42}";
    RosType check_message;
    check_message.data = 42424242424242.42;
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message, check_func));
  }
}

TEST(JsonToRos2Message, BasicType_Int8) {
  using RosType = ::example_interfaces::msg::Int8;

  {
    std::string json_str = "{\"data\":127}";
    RosType check_message;
    check_message.data = std::numeric_limits<int8_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":-128}";
    RosType check_message;
    check_message.data = std::numeric_limits<int8_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_UInt8) {
  using RosType = ::example_interfaces::msg::UInt8;

  {
    std::string json_str = "{\"data\":255}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint8_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":0}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint8_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_Int16) {
  using RosType = ::example_interfaces::msg::Int16;

  {
    std::string json_str = "{\"data\":32767}";
    RosType check_message;
    check_message.data = std::numeric_limits<int16_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":-32768}";
    RosType check_message;
    check_message.data = std::numeric_limits<int16_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_UInt16) {
  using RosType = ::example_interfaces::msg::UInt16;

  {
    std::string json_str = "{\"data\":65535}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint16_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":0}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint16_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_Int32) {
  using RosType = ::example_interfaces::msg::Int32;

  {
    std::string json_str = "{\"data\":2147483647}";
    RosType check_message;
    check_message.data = std::numeric_limits<int32_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":-2147483648}";
    RosType check_message;
    check_message.data = std::numeric_limits<int32_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_UInt32) {
  using RosType = ::example_interfaces::msg::UInt32;

  {
    std::string json_str = "{\"data\":4294967295}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint32_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":0}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint32_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_Int64) {
  using RosType = ::example_interfaces::msg::Int64;

  {
    std::string json_str = "{\"data\":9223372036854775807}";
    RosType check_message;
    check_message.data = std::numeric_limits<int64_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":-9223372036854775808}";
    RosType check_message;
    check_message.data = std::numeric_limits<int64_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

TEST(JsonToRos2Message, BasicType_UInt64) {
  using RosType = ::example_interfaces::msg::UInt64;

  {
    std::string json_str = "{\"data\":18446744073709551615}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint64_t>::max();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }

  {
    std::string json_str = "{\"data\":0}";
    RosType check_message;
    check_message.data = std::numeric_limits<uint64_t>::min();
    EXPECT_TRUE(TestJsonToRos2Message<RosType>(json_str, check_message));
  }
}

int CheckBasicType(const test_msgs::msg::BasicTypes& first, const test_msgs::msg::BasicTypes& second) {
  if (first.bool_value != second.bool_value) return -1;
  if (first.byte_value != second.byte_value) return -2;
  if (first.char_value != second.char_value) return -3;
  if (first.float32_value != second.float32_value) return -4;
  if (first.float64_value != second.float64_value) return -5;
  if (first.int8_value != second.int8_value) return -6;
  if (first.uint8_value != second.uint8_value) return -7;
  if (first.int16_value != second.int16_value) return -8;
  if (first.uint16_value != second.uint16_value) return -9;
  if (first.int32_value != second.int32_value) return -10;
  if (first.uint32_value != second.uint32_value) return -11;
  if (first.int64_value != second.int64_value) return -12;
  if (first.uint64_value != second.uint64_value) return -13;
  return 0;
}

#define CheckFirstSecondArray(type)                                   \
  EXPECT_EQ(first.type##_values.size(), second.type##_values.size()); \
  for (size_t i = 0; i < first.type##_values.size(); ++i) {           \
    EXPECT_EQ(first.type##_values[i], second.type##_values[i]);       \
  }

template <class ArrayType>
void CheckArrayMsg(const ArrayType& first, const ArrayType& second) {
  // basic array
  EXPECT_EQ(first.basic_types_values.size(), second.basic_types_values.size());
  for (size_t i = 0; i < first.basic_types_values.size(); ++i) {
    EXPECT_EQ(CheckBasicType(first.basic_types_values[i], second.basic_types_values[i]), 0);
  }
  CheckFirstSecondArray(bool);
  CheckFirstSecondArray(byte);
  CheckFirstSecondArray(char);
  CheckFirstSecondArray(float32);
  CheckFirstSecondArray(float64);
  CheckFirstSecondArray(int8);
  CheckFirstSecondArray(uint8);
  CheckFirstSecondArray(int16);
  CheckFirstSecondArray(uint16);
  CheckFirstSecondArray(int32);
  CheckFirstSecondArray(uint32);
  CheckFirstSecondArray(int64);
  CheckFirstSecondArray(uint64);
  CheckFirstSecondArray(string);
}

TEST(ArrayTypeToMessage, FixedSizeArray) {
  auto array_list = GetMessagesArrays();
  for (auto one_array : array_list) {
    const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::Arrays>();
    std::string json_str;
    bool ret = MessageToJson(reinterpret_cast<void*>(one_array.get()), type_support, json_str);
    ASSERT_TRUE(ret);

    test_msgs::msg::Arrays deserialize_message;
    ret = JsonToMessage(json_str, type_support, &deserialize_message);
    ASSERT_TRUE(ret);

    CheckArrayMsg(*one_array, deserialize_message);
  }
}

TEST(BoundTypeToMessage, BoundMaxSizeArray) {
  auto array_list = GetMessagesBoundedSequences();
  for (auto one_array : array_list) {
    const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::BoundedSequences>();
    std::string json_str;
    bool ret = MessageToJson(reinterpret_cast<void*>(one_array.get()), type_support, json_str);
    ASSERT_TRUE(ret);

    test_msgs::msg::BoundedSequences deserialize_message;
    ret = JsonToMessage(json_str, type_support, &deserialize_message);
    ASSERT_TRUE(ret);

    CheckArrayMsg(*one_array, deserialize_message);
  }
}

TEST(BoundTypeToMessage, UnBoundMaxSizeArray) {
  auto array_list = GetMessagesUnboundedSequences();
  for (auto one_array : array_list) {
    const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::UnboundedSequences>();
    std::string json_str;
    bool ret = MessageToJson(reinterpret_cast<void*>(one_array.get()), type_support, json_str);
    ASSERT_TRUE(ret);

    test_msgs::msg::UnboundedSequences deserialize_message;
    ret = JsonToMessage(json_str, type_support, &deserialize_message);
    ASSERT_TRUE(ret);

    CheckArrayMsg(*one_array, deserialize_message);
  }
}

TEST(NestedTypeToMessage, SingleNest) {
  auto nest_list = GetMessagesNested();
  for (auto one_nest : nest_list) {
    const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::Nested>();
    std::string json_str;
    bool ret = MessageToJson(reinterpret_cast<void*>(one_nest.get()), type_support, json_str);
    ASSERT_TRUE(ret);

    test_msgs::msg::Nested deserialize_message;
    ret = JsonToMessage(json_str, type_support, &deserialize_message);
    ASSERT_TRUE(ret);

    CheckBasicType(one_nest->basic_types_value, deserialize_message.basic_types_value);
  }
}

TEST(NestedTypeToMessage, MultiNest) {
  auto nest_list = GetMessagesMultiNested();
  for (auto one_nest : nest_list) {
    const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::MultiNested>();
    std::string json_str;
    bool ret = MessageToJson(reinterpret_cast<void*>(one_nest.get()), type_support, json_str);
    ASSERT_TRUE(ret);

    test_msgs::msg::MultiNested deserialize_message;
    ret = JsonToMessage(json_str, type_support, &deserialize_message);
    ASSERT_TRUE(ret);

    // array
    EXPECT_EQ(one_nest->array_of_arrays.size(), deserialize_message.array_of_arrays.size());
    for (int i = 0; i < one_nest->array_of_arrays.size(); i++) {
      CheckArrayMsg(one_nest->array_of_arrays[i], deserialize_message.array_of_arrays[i]);
    }
    // bounded
    EXPECT_EQ(one_nest->array_of_bounded_sequences.size(), deserialize_message.array_of_bounded_sequences.size());
    for (int i = 0; i < one_nest->array_of_bounded_sequences.size(); i++) {
      CheckArrayMsg(one_nest->array_of_bounded_sequences[i], deserialize_message.array_of_bounded_sequences[i]);
    }
    EXPECT_EQ(one_nest->array_of_unbounded_sequences.size(), deserialize_message.array_of_unbounded_sequences.size());
    for (int i = 0; i < one_nest->array_of_unbounded_sequences.size(); i++) {
      CheckArrayMsg(one_nest->array_of_unbounded_sequences[i], deserialize_message.array_of_unbounded_sequences[i]);
    }
    EXPECT_EQ(one_nest->bounded_sequence_of_bounded_sequences.size(), deserialize_message.bounded_sequence_of_bounded_sequences.size());
    for (int i = 0; i < one_nest->bounded_sequence_of_bounded_sequences.size(); i++) {
      CheckArrayMsg(one_nest->bounded_sequence_of_bounded_sequences[i], deserialize_message.bounded_sequence_of_bounded_sequences[i]);
    }
    EXPECT_EQ(one_nest->unbounded_sequence_of_arrays.size(), deserialize_message.unbounded_sequence_of_arrays.size());
    for (int i = 0; i < one_nest->unbounded_sequence_of_arrays.size(); i++) {
      CheckArrayMsg(one_nest->unbounded_sequence_of_arrays[i], deserialize_message.unbounded_sequence_of_arrays[i]);
    }
    EXPECT_EQ(one_nest->unbounded_sequence_of_bounded_sequences.size(), deserialize_message.unbounded_sequence_of_bounded_sequences.size());
    for (int i = 0; i < one_nest->unbounded_sequence_of_bounded_sequences.size(); i++) {
      CheckArrayMsg(one_nest->unbounded_sequence_of_bounded_sequences[i], deserialize_message.unbounded_sequence_of_bounded_sequences[i]);
    }
    EXPECT_EQ(one_nest->unbounded_sequence_of_unbounded_sequences.size(), deserialize_message.unbounded_sequence_of_unbounded_sequences.size());
    for (int i = 0; i < one_nest->unbounded_sequence_of_unbounded_sequences.size(); i++) {
      CheckArrayMsg(one_nest->unbounded_sequence_of_unbounded_sequences[i], deserialize_message.unbounded_sequence_of_unbounded_sequences[i]);
    }
  }
}

TEST(Ros2MessageToJson, BasicType_Bool) {
  using RosType = ::example_interfaces::msg::Bool;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = true;

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":true}\n");
  }

  {
    RosType msg;
    msg.data = false;

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":false}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_Float32) {
  using RosType = ::example_interfaces::msg::Float32;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = 65.34f;

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);

    RosType msg2;
    ret = JsonToMessage(json_str, type_support, &msg2);
    ASSERT_TRUE(ret);

    EXPECT_NEAR(msg2.data, msg.data, 1e-6);
  }

  {
    RosType msg;
    msg.data = -65.34f;

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);

    RosType msg2;
    ret = JsonToMessage(json_str, type_support, &msg2);
    ASSERT_TRUE(ret);

    EXPECT_NEAR(msg2.data, msg.data, 1e-6);
  }
}

TEST(Ros2MessageToJson, BasicType_Float64) {
  using RosType = ::example_interfaces::msg::Float64;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = 65656565656565.34;

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);

    RosType msg2;
    ret = JsonToMessage(json_str, type_support, &msg2);
    ASSERT_TRUE(ret);

    EXPECT_NEAR(msg2.data, msg.data, 1e-6);
  }

  {
    RosType msg;
    msg.data = -65656565656565.34;

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);

    RosType msg2;
    ret = JsonToMessage(json_str, type_support, &msg2);
    ASSERT_TRUE(ret);

    EXPECT_NEAR(msg2.data, msg.data, 1e-6);
  }
}

TEST(Ros2MessageToJson, BasicType_Int8) {
  using RosType = ::example_interfaces::msg::Int8;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<int8_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":127}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<int8_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":-128}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_Uint8) {
  using RosType = ::example_interfaces::msg::UInt8;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<uint8_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":255}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<uint8_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":0}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_Int16) {
  using RosType = ::example_interfaces::msg::Int16;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<int16_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":32767}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<int16_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":-32768}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_UInt16) {
  using RosType = ::example_interfaces::msg::UInt16;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<uint16_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":65535}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<uint16_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":0}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_Int32) {
  using RosType = ::example_interfaces::msg::Int32;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<int32_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":2147483647}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<int32_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":-2147483648}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_UInt32) {
  using RosType = ::example_interfaces::msg::UInt32;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<uint32_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":4294967295}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<uint32_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":0}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_Int64) {
  using RosType = ::example_interfaces::msg::Int64;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<int64_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":9223372036854775807}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<int64_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":-9223372036854775808}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_UInt64) {
  using RosType = ::example_interfaces::msg::UInt64;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = std::numeric_limits<uint64_t>::max();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":18446744073709551615}\n");
  }

  {
    RosType msg;
    msg.data = std::numeric_limits<uint64_t>::min();

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":0}\n");
  }
}

TEST(Ros2MessageToJson, BasicType_String) {
  using RosType = ::example_interfaces::msg::String;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    RosType msg;
    msg.data = "testsdfggsdedasdggfhdascdgertysras";

    std::string json_str;
    bool ret = MessageToJson(&msg, type_support, json_str);
    ASSERT_TRUE(ret);
    EXPECT_STREQ(json_str.c_str(), "{\"data\":\"testsdfggsdedasdggfhdascdgertysras\"}\n");
  }
}

#define CHECK_BASIC_TYPES(type, inner_type)   \
  EXPECT_TRUE(item.isMember(#type "_value")); \
  EXPECT_EQ(GetJsonValue<inner_type>(item[#type "_value"]), ros_item.type##_value);

#define CHECK_ARRAY_TYPE_VALUES(type, inner_type)                                              \
  EXPECT_TRUE(json_node.isMember(#type "_values"));                                            \
  EXPECT_TRUE(json_node[#type "_values"].size() == msgs.type##_values.size());                 \
  for (int i = 0; i < msgs.type##_values.size(); i++) {                                        \
    EXPECT_EQ(GetJsonValue<inner_type>(json_node[#type "_values"][i]), msgs.type##_values[i]); \
  }

template <typename ArrayType>
void CheckArrays(Json::Value& json_node, const ArrayType& msgs) {
  EXPECT_TRUE(json_node.isMember("basic_types_values"));
  EXPECT_TRUE(json_node["basic_types_values"].size() == msgs.basic_types_values.size());
  for (int i = 0; i < json_node["basic_types_values"].size(); i++) {
    auto item = json_node["basic_types_values"][i];
    auto ros_item = msgs.basic_types_values[i];

    CHECK_BASIC_TYPES(bool, bool);
    CHECK_BASIC_TYPES(byte, uint8_t);
    CHECK_BASIC_TYPES(char, uint8_t);
    CHECK_BASIC_TYPES(float32, float);
    CHECK_BASIC_TYPES(float64, double);
    CHECK_BASIC_TYPES(int8, int8_t);
    CHECK_BASIC_TYPES(uint8, uint8_t);
    CHECK_BASIC_TYPES(int16, int16_t);
    CHECK_BASIC_TYPES(uint16, uint16_t);
    CHECK_BASIC_TYPES(int32, int32_t);
    CHECK_BASIC_TYPES(uint32, uint32_t);
    CHECK_BASIC_TYPES(int64, int64_t);
    CHECK_BASIC_TYPES(uint64, uint64_t);
  }

  CHECK_ARRAY_TYPE_VALUES(bool, bool);
  CHECK_ARRAY_TYPE_VALUES(byte, uint8_t);
  CHECK_ARRAY_TYPE_VALUES(char, uint8_t);
  CHECK_ARRAY_TYPE_VALUES(float32, float);
  CHECK_ARRAY_TYPE_VALUES(float64, double);
  CHECK_ARRAY_TYPE_VALUES(int8, int8_t);
  CHECK_ARRAY_TYPE_VALUES(uint8, uint8_t);
  CHECK_ARRAY_TYPE_VALUES(int16, int16_t);
  CHECK_ARRAY_TYPE_VALUES(uint16, uint16_t);
  CHECK_ARRAY_TYPE_VALUES(int32, int32_t);
  CHECK_ARRAY_TYPE_VALUES(uint32, uint32_t);
  CHECK_ARRAY_TYPE_VALUES(int64, int64_t);
  CHECK_ARRAY_TYPE_VALUES(uint64, uint64_t);
  CHECK_ARRAY_TYPE_VALUES(string, std::string);
}

TEST(ArrayTypeToJson, FixSizeArrays) {
  auto msg = GetMessagesArrays();
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::Arrays>();
  for (auto one_msg : msg) {
    std::string json_str;
    MessageToJson(one_msg.get(), type_support, json_str);

    Json::Reader reader;
    Json::Value json_node;
    reader.parse(json_str, json_node);
    CheckArrays(json_node, *one_msg);
  }
}

TEST(ArrayTypeToJson, BoundedSizeArray) {
  auto msg = GetMessagesBoundedSequences();
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::BoundedSequences>();
  for (auto one_msg : msg) {
    std::string json_str;
    MessageToJson(one_msg.get(), type_support, json_str);

    Json::Reader reader;
    Json::Value json_node;
    reader.parse(json_str, json_node);
    CheckArrays(json_node, *one_msg);
  }
}

TEST(ArrayTypeToJson, UnboundedSizeArray) {
  auto msg = GetMessagesUnboundedSequences();
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::UnboundedSequences>();
  for (auto one_msg : msg) {
    std::string json_str;
    MessageToJson(one_msg.get(), type_support, json_str);

    Json::Reader reader;
    Json::Value json_node;
    reader.parse(json_str, json_node);
    CheckArrays(json_node, *one_msg);
  }
}

#define CHECK_BOUNDED_ARRAY_BASIC_TYPES(type, inner_type)            \
  EXPECT_TRUE(json_basic_types_value_node.isMember(#type "_value")); \
  EXPECT_EQ(GetJsonValue<inner_type>(json_basic_types_value_node[#type "_value"]), basic_types_values.type##_value);

// ros msg
TEST(NestTypeToJson, Nest) {
  std::vector<test_msgs::msg::Nested::SharedPtr> msg = GetMessagesNested();
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::Nested>();
  for (auto one_msg : msg) {
    std::string json_str;
    MessageToJson(one_msg.get(), type_support, json_str);

    Json::Reader reader;
    Json::Value json_node;
    reader.parse(json_str, json_node);

    // basic value
    Json::Value json_basic_types_value_node = json_node["basic_types_value"];
    auto basic_types_values = one_msg->basic_types_value;
    EXPECT_TRUE(json_node.isMember("basic_types_value"));

    CHECK_BOUNDED_ARRAY_BASIC_TYPES(bool, bool);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(byte, uint8_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(char, uint8_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(float32, float);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(float64, double);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(int8, int8_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(uint8, uint8_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(int16, int16_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(uint16, uint16_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(int32, int32_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(uint32, uint32_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(int64, int64_t);
    CHECK_BOUNDED_ARRAY_BASIC_TYPES(uint64, uint64_t);
  }
}

TEST(NestTypeToJson, MultiNest) {
  std::vector<test_msgs::msg::MultiNested::SharedPtr> multi_nest_msg = GetMessagesMultiNested();
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<test_msgs::msg::MultiNested>();
  for (auto one_nest_msg : multi_nest_msg) {
    std::string json_str;
    MessageToJson(one_nest_msg.get(), type_support, json_str);

    Json::Reader reader;
    Json::Value json_node;
    reader.parse(json_str, json_node);

    // test_msgs::msg::Arrays
    EXPECT_TRUE(json_node.isMember("array_of_arrays"));
    EXPECT_TRUE(json_node["array_of_arrays"].size() == one_nest_msg->array_of_arrays.size());
    for (int i = 0; i < one_nest_msg->array_of_arrays.size(); i++) {
      CheckArrays(json_node["array_of_arrays"][i], one_nest_msg->array_of_arrays[i]);
    }
    // array test_msgs::msg::BoundedSequences
    EXPECT_TRUE(json_node.isMember("array_of_bounded_sequences"));
    EXPECT_TRUE(json_node["array_of_bounded_sequences"].size() == one_nest_msg->array_of_bounded_sequences.size());
    for (int i = 0; i < one_nest_msg->array_of_bounded_sequences.size(); i++) {
      CheckArrays(json_node["array_of_bounded_sequences"][i], one_nest_msg->array_of_bounded_sequences[i]);
    }
    // array test_msgs::msg::UnBoundedSequences
    EXPECT_TRUE(json_node.isMember("array_of_unbounded_sequences"));
    EXPECT_TRUE(json_node["array_of_unbounded_sequences"].size() == one_nest_msg->array_of_unbounded_sequences.size());
    for (int i = 0; i < one_nest_msg->array_of_unbounded_sequences.size(); i++) {
      CheckArrays(json_node["array_of_unbounded_sequences"][i], one_nest_msg->array_of_unbounded_sequences[i]);
    }

    // sized test_msgs::msg::BoundedSequences
    EXPECT_TRUE(json_node.isMember("bounded_sequence_of_bounded_sequences"));
    EXPECT_TRUE(json_node["bounded_sequence_of_bounded_sequences"].size() == one_nest_msg->bounded_sequence_of_bounded_sequences.size());
    for (int i = 0; i < one_nest_msg->bounded_sequence_of_bounded_sequences.size(); i++) {
      CheckArrays(json_node["bounded_sequence_of_bounded_sequences"][i], one_nest_msg->bounded_sequence_of_bounded_sequences[i]);
    }
    // sized test_msgs::msg::UnBoundedSequences
    EXPECT_TRUE(json_node.isMember("bounded_sequence_of_unbounded_sequences"));
    EXPECT_TRUE(json_node["bounded_sequence_of_unbounded_sequences"].size() == one_nest_msg->bounded_sequence_of_unbounded_sequences.size());
    for (int i = 0; i < one_nest_msg->bounded_sequence_of_unbounded_sequences.size(); i++) {
      CheckArrays(json_node["bounded_sequence_of_unbounded_sequences"][i], one_nest_msg->bounded_sequence_of_unbounded_sequences[i]);
    }

    // unsized test_msgs::msg::BoundedSequences
    EXPECT_TRUE(json_node.isMember("unbounded_sequence_of_bounded_sequences"));
    EXPECT_TRUE(json_node["unbounded_sequence_of_bounded_sequences"].size() == one_nest_msg->unbounded_sequence_of_bounded_sequences.size());
    for (int i = 0; i < one_nest_msg->unbounded_sequence_of_bounded_sequences.size(); i++) {
      CheckArrays(json_node["unbounded_sequence_of_bounded_sequences"][i], one_nest_msg->unbounded_sequence_of_bounded_sequences[i]);
    }
    // unsized test_msgs::msg::UnBoundedSequences
    EXPECT_TRUE(json_node.isMember("unbounded_sequence_of_unbounded_sequences"));
    EXPECT_TRUE(json_node["unbounded_sequence_of_unbounded_sequences"].size() == one_nest_msg->unbounded_sequence_of_unbounded_sequences.size());
    for (int i = 0; i < one_nest_msg->unbounded_sequence_of_unbounded_sequences.size(); i++) {
      CheckArrays(json_node["unbounded_sequence_of_unbounded_sequences"][i], one_nest_msg->unbounded_sequence_of_unbounded_sequences[i]);
    }
  }
}

}  // namespace aimrt::common::ros2_util