// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/parameter/parameter_handle.h"
#include <gtest/gtest.h>

namespace aimrt::runtime::core::parameter {

ParameterHandle parameter_handle;

TEST(ParameterHandleTest, SetParameter) {
  EXPECT_EQ(parameter_handle.GetParameter("test_parameter1"), nullptr);
  parameter_handle.SetParameter("test_parameter1", "test_value1");
  EXPECT_EQ(*parameter_handle.GetParameter("test_parameter1"), "test_value1");
}

TEST(ParameterHandleTest, ListParameter) {
  parameter_handle.SetParameter("test_parameter1", "test_value1");
  parameter_handle.SetParameter("test_parameter2", "test_value2");
  parameter_handle.SetParameter("test_parameter3", "test_value3");
  EXPECT_EQ(parameter_handle.ListParameter().size(), 3);
}
}  // namespace aimrt::runtime::core::parameter