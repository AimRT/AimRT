// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include <cmath>
#include <test_msgs/msg/detail/nested__struct.hpp>

#include "ros2_util/yaml_convert.h"

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

template <class RosType>
bool TestYamlToRos2Message(
    const std::string& yaml_str,
    const RosType& check_message,
    std::function<bool(const RosType&, const RosType&)> check_func =
        [](const RosType& lhs, const RosType& rhs) -> bool { return lhs == rhs; }) {
  RosType message;
  bool ret = YamlToMessage(yaml_str, GetIntrospectionTypeSupport<RosType>(), &message);
  if (!ret) return false;
  return check_func(message, check_message);
}

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
AlmostEqual(T x, T y, int ulp) {
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp ||
         std::fabs(x - y) < std::numeric_limits<T>::min();
}

TEST(YamlToRos2Message, BasicType_Bool) {
  using RosType = ::example_interfaces::msg::Bool;

  {
    std::string yaml_str = "data: true";
    RosType check_message;
    check_message.data = true;
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: false";
    RosType check_message;
    check_message.data = false;
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_Float32) {
  using RosType = ::example_interfaces::msg::Float32;

  auto check_func = [](const RosType& lhs, const RosType& rhs) -> bool {
    return AlmostEqual(lhs.data, rhs.data, 2);
  };

  {
    std::string yaml_str = "data: 3.14159";
    RosType check_message;
    check_message.data = 3.14159f;
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: -2.71828";
    RosType check_message;
    check_message.data = -2.71828f;
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_Float64) {
  using RosType = ::example_interfaces::msg::Float64;

  auto check_func = [](const RosType& lhs, const RosType& rhs) -> bool {
    return AlmostEqual(lhs.data, rhs.data, 2);
  };

  {
    std::string yaml_str = "data: 42424242424242.42";
    RosType check_message;
    check_message.data = 42424242424242.42;
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message, check_func));
  }
}

TEST(YamlToRos2Message, BasicType_Int8) {
  using RosType = ::example_interfaces::msg::Int8;
  {
    std::string yaml_str = "data: 127";  // max value
    RosType check_message;
    check_message.data = std::numeric_limits<int8_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: -128";  // min value
    RosType check_message;
    check_message.data = std::numeric_limits<int8_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_UInt8) {
  using RosType = ::example_interfaces::msg::UInt8;
  {
    std::string yaml_str = "data: 255";  // max value
    RosType check_message;
    check_message.data = std::numeric_limits<uint8_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: 0";  // min value
    RosType check_message;
    check_message.data = std::numeric_limits<uint8_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_Int16) {
  using RosType = ::example_interfaces::msg::Int16;

  {
    std::string yaml_str = "data: 32767";
    RosType check_message;
    check_message.data = std::numeric_limits<int16_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: -32768";
    RosType check_message;
    check_message.data = std::numeric_limits<int16_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_UInt16) {
  using RosType = ::example_interfaces::msg::UInt16;

  {
    std::string yaml_str = "data: 65535";
    RosType check_message;
    check_message.data = std::numeric_limits<uint16_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: 0";
    RosType check_message;
    check_message.data = std::numeric_limits<uint16_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_Int32) {
  using RosType = ::example_interfaces::msg::Int32;

  {
    std::string yaml_str = "data: 2147483647";
    RosType check_message;
    check_message.data = std::numeric_limits<int32_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: -2147483648";
    RosType check_message;
    check_message.data = std::numeric_limits<int32_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_UInt32) {
  using RosType = ::example_interfaces::msg::UInt32;

  {
    std::string yaml_str = "data: 4294967295";
    RosType check_message;
    check_message.data = std::numeric_limits<uint32_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: 0";
    RosType check_message;
    check_message.data = std::numeric_limits<uint32_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_Int64) {
  using RosType = ::example_interfaces::msg::Int64;

  {
    std::string yaml_str = "data: 9223372036854775807";
    RosType check_message;
    check_message.data = std::numeric_limits<int64_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: -9223372036854775808";
    RosType check_message;
    check_message.data = std::numeric_limits<int64_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_UInt64) {
  using RosType = ::example_interfaces::msg::UInt64;

  {
    std::string yaml_str = "data: 18446744073709551615";
    RosType check_message;
    check_message.data = std::numeric_limits<uint64_t>::max();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }

  {
    std::string yaml_str = "data: 0";
    RosType check_message;
    check_message.data = std::numeric_limits<uint64_t>::min();
    EXPECT_TRUE(TestYamlToRos2Message<RosType>(yaml_str, check_message));
  }
}

TEST(YamlToRos2Message, BasicType_String) {
  using RosType = ::example_interfaces::msg::String;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  {
    std::string yaml_str = "data: \"Hello World\"";
    RosType msg;
    bool ret = YamlToMessage(yaml_str, type_support, &msg);
    ASSERT_TRUE(ret);
    EXPECT_EQ(msg.data, "Hello World");
  }

  {
    std::string yaml_str = "data: \"\"";  // empty string
    RosType msg;
    bool ret = YamlToMessage(yaml_str, type_support, &msg);
    ASSERT_TRUE(ret);
    EXPECT_TRUE(msg.data.empty());
  }
}

TEST(YamlToRos2Message, BasicTypes_Complex) {
  using RosType = ::test_msgs::msg::BasicTypes;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();

  std::string yaml_str = R"(
bool_value: true
byte_value: 255
char_value: 100
float32_value: 1.125
float64_value: -2.125
int8_value: 127
uint8_value: 255
int16_value: 32767
uint16_value: 65535
int32_value: 2147483647
uint32_value: 4294967295
int64_value: 9223372036854775807
uint64_value: 18446744073709551615
  )";

  RosType msg;
  bool ret = YamlToMessage(yaml_str, type_support, &msg);
  ASSERT_TRUE(ret);

  EXPECT_TRUE(msg.bool_value);
  EXPECT_EQ(msg.byte_value, 255);
  EXPECT_EQ(msg.char_value, 100);
  EXPECT_FLOAT_EQ(msg.float32_value, 1.125f);
  EXPECT_DOUBLE_EQ(msg.float64_value, -2.125);
  EXPECT_EQ(msg.int8_value, 127);
  EXPECT_EQ(msg.uint8_value, 255);
  EXPECT_EQ(msg.int16_value, 32767);
  EXPECT_EQ(msg.uint16_value, 65535);
  EXPECT_EQ(msg.int32_value, 2147483647);
  EXPECT_EQ(msg.uint32_value, 4294967295);
  EXPECT_EQ(msg.int64_value, 9223372036854775807);
  EXPECT_EQ(msg.uint64_value, 18446744073709551615ULL);
}

TEST(YamlToRos2Message, SingleNest) {
  using RosType = ::test_msgs::msg::Nested;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();
  std::string yaml_str = R"(
  basic_types_value:
    int8_value: 127
    int16_value: 32767
    int32_value: 2147483647
    int64_value: 9223372036854775807
  )";

  RosType msg;
  bool ret = YamlToMessage(yaml_str, type_support, &msg);
  ASSERT_TRUE(ret);
  EXPECT_EQ(msg.basic_types_value.int8_value, 127);
  EXPECT_EQ(msg.basic_types_value.int16_value, 32767);
  EXPECT_EQ(msg.basic_types_value.int32_value, 2147483647);
  EXPECT_EQ(msg.basic_types_value.int64_value, 9223372036854775807);
}

TEST(YamlToRos2Message, MultipleNest) {
  using RosType = ::test_msgs::msg::MultiNested;
  const rosidl_message_type_support_t* type_support = GetIntrospectionTypeSupport<RosType>();
  std::string yaml_str = R"(
  array_of_arrays:
    - bool_values: [true, false, true]
      byte_values: [255, 0, 255]
      int32_values: [2147483647, -2147483648, 0]
      string_values: ["test1", "test2", "test3"]
      basic_types_values:
        - bool_value: true
          int32_value: 42
        - bool_value: false
          int32_value: -42
        - bool_value: true
          int32_value: 0
  array_of_bounded_sequences:
    - bool_values: [true, false]
      int32_values: [42, -42]
      string_values: ["bounded1", "bounded2"]
    - bool_values: [false, true]
      int32_values: [-1, 1]
      string_values: ["bounded3", "bounded4"]
    - bool_values: [true, true]
      int32_values: [0, 0]
      string_values: ["bounded5", "bounded6"]
  )";

  RosType msg;
  bool ret = YamlToMessage(yaml_str, type_support, &msg);
  ASSERT_TRUE(ret);

  // test  array_of_arrays
  ASSERT_EQ(msg.array_of_arrays.size(), 3u);
  EXPECT_TRUE(msg.array_of_arrays[0].bool_values[0]);
  EXPECT_FALSE(msg.array_of_arrays[0].bool_values[1]);
  EXPECT_TRUE(msg.array_of_arrays[0].bool_values[2]);

  EXPECT_EQ(msg.array_of_arrays[0].byte_values[0], 255);
  EXPECT_EQ(msg.array_of_arrays[0].byte_values[1], 0);
  EXPECT_EQ(msg.array_of_arrays[0].byte_values[2], 255);

  EXPECT_EQ(msg.array_of_arrays[0].int32_values[0], 2147483647);
  EXPECT_EQ(msg.array_of_arrays[0].int32_values[1], -2147483648);
  EXPECT_EQ(msg.array_of_arrays[0].int32_values[2], 0);

  EXPECT_EQ(msg.array_of_arrays[0].string_values[0], "test1");
  EXPECT_EQ(msg.array_of_arrays[0].string_values[1], "test2");
  EXPECT_EQ(msg.array_of_arrays[0].string_values[2], "test3");

  EXPECT_EQ(msg.array_of_arrays[0].basic_types_values[0].bool_value, true);
  EXPECT_EQ(msg.array_of_arrays[0].basic_types_values[0].int32_value, 42);
  EXPECT_EQ(msg.array_of_arrays[0].basic_types_values[1].bool_value, false);
  EXPECT_EQ(msg.array_of_arrays[0].basic_types_values[1].int32_value, -42);
  EXPECT_EQ(msg.array_of_arrays[0].basic_types_values[2].bool_value, true);
  EXPECT_EQ(msg.array_of_arrays[0].basic_types_values[2].int32_value, 0);

  // test array_of_bounded_sequences
  ASSERT_EQ(msg.array_of_bounded_sequences.size(), 3u);
  EXPECT_TRUE(msg.array_of_bounded_sequences[0].bool_values[0]);
  EXPECT_FALSE(msg.array_of_bounded_sequences[0].bool_values[1]);

  EXPECT_EQ(msg.array_of_bounded_sequences[0].int32_values[0], 42);
  EXPECT_EQ(msg.array_of_bounded_sequences[0].int32_values[1], -42);

  EXPECT_EQ(msg.array_of_bounded_sequences[0].string_values[0], "bounded1");
  EXPECT_EQ(msg.array_of_bounded_sequences[0].string_values[1], "bounded2");
}
}  // namespace aimrt::common::ros2_util
