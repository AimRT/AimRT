// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>
#include <ros2_plugin/ros2_rpc_backend.h>

namespace aimrt::plugins::ros2_plugin {

// Test the GetRemappedFuncName function
TEST(ROS2_RPC_BACKEND_TEST, GetRemappedFuncName_test) {
  std::string input_string_pb = "pb:/test_pb_service/test_pb_func";
  std::string input_string_ros2 = "ros2:/test_ros2_service/test_ros2_func";

  // Test cases with wildcard matching with msg_type
  std::string matching_rule_1 = "(.*)/(.*)/(.*)";
  std::string remapping_rule_1 = "{1}/{3}";
  std::string ret_pb_1 = Ros2RpcBackend::GetRemappedFuncName(input_string_pb, matching_rule_1, remapping_rule_1);
  std::string ret_ros2_1 = Ros2RpcBackend::GetRemappedFuncName(input_string_ros2, matching_rule_1, remapping_rule_1);
  EXPECT_EQ(ret_pb_1, "pb:/test_pb_func");
  EXPECT_EQ(ret_ros2_1, "ros2:/test_ros2_func");

  // Test cases with wildcard matching without msg_type
  std::string matching_rule_2 = "(.*)/(.*)/(.*)";
  std::string remapping_rule_2 = "/{3}";
  std::string ret_pb_2 = Ros2RpcBackend::GetRemappedFuncName(input_string_pb, matching_rule_2, remapping_rule_2);
  std::string ret_ros2_2 = Ros2RpcBackend::GetRemappedFuncName(input_string_ros2, matching_rule_2, remapping_rule_2);
  EXPECT_EQ(ret_pb_2, "pb:/test_pb_func");
  EXPECT_EQ(ret_ros2_2, "ros2:/test_ros2_func");

  // Test cases with specific matching
  std::string matching_rule_3 = "(.*)/(test_pb_service)/(.*)";
  std::string remapping_rule_3 = "/{2}/test_pb_func_new";
  std::string ret_pb_3 = Ros2RpcBackend::GetRemappedFuncName(input_string_pb, matching_rule_3, remapping_rule_3);
  EXPECT_EQ(ret_pb_3, "pb:/test_pb_service/test_pb_func_new");

  // Test cases with empty matching
  std::string matching_rule_4;
  std::string ret_pb_4 = Ros2RpcBackend::GetRemappedFuncName(input_string_pb, matching_rule_4, remapping_rule_3);
  EXPECT_EQ(ret_pb_4, "pb:/test_pb_service/test_pb_func");

  // Test cases with empty remapping
  std::string remapping_rule_5;
  std::string ret_pb_5 = Ros2RpcBackend::GetRemappedFuncName(input_string_pb, matching_rule_3, remapping_rule_5);
  EXPECT_EQ(ret_pb_5, "pb:/test_pb_service/test_pb_func");

  // Test cases with non-matching regex
  std::string matching_rule_6 = "abcdefg";
  std::string remapping_rule_6 = "{1}/{2}/{3}";
  std::string ret_pb_6 = Ros2RpcBackend::GetRemappedFuncName(input_string_pb, matching_rule_6, remapping_rule_6);
  EXPECT_EQ(ret_pb_6, "pb:/test_pb_service/test_pb_func");

  // Test cases out of range index in remapping rule
  std::string matching_rule_7 = "(.*)/(.*)/(.*)";
  std::string remapping_rule_7 = "{4}";
  std::string ret_pb_7 = Ros2RpcBackend::GetRemappedFuncName(input_string_pb, matching_rule_7, remapping_rule_7);
  EXPECT_EQ(ret_pb_7, "pb:/test_pb_service/test_pb_func");
}

}  // namespace aimrt::plugins::ros2_plugin