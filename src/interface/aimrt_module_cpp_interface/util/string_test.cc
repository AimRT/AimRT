// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "aimrt_module_cpp_interface/util/string.h"

namespace aimrt::util {
// StdString -> AimRTStringView
TEST(StringConversionTest, ToAimRTStringViewFromStdString) {
  std::string str = "test";
  aimrt_string_view_t view = ToAimRTStringView(str);
  EXPECT_EQ(view.str, str.c_str());
  EXPECT_EQ(view.len, str.size());
}
// StdStringView -> AimRTStringView
TEST(StringConversionTest, ToAimRTStringViewFromStdStringView) {
  std::string_view str_view = "test";
  aimrt_string_view_t view = ToAimRTStringView(str_view);
  EXPECT_EQ(view.str, str_view.data());
  EXPECT_EQ(view.len, str_view.size());
}

// CharPtr -> AimRTStringView
TEST(StringConversionTest, ToAimRTStringViewFromCharPtr) {
  const char* str = "test";
  aimrt_string_view_t view = ToAimRTStringView(str);
  EXPECT_EQ(view.str, str);
  EXPECT_EQ(view.len, strlen(str));
}

// AimRTStringView -> StdStringView
TEST(StringConversionTest, ToStdStringView) {
  aimrt_string_view_t aimrt_view = {"test", 4};
  std::string_view str_view = ToStdStringView(aimrt_view);
  EXPECT_EQ(str_view.data(), aimrt_view.str);
  EXPECT_EQ(str_view.size(), aimrt_view.len);
}

// AimRTStringView -> StdString
TEST(StringConversionTest, ToStdString) {
  aimrt_string_view_t aimrt_view = {"test", 4};
  std::string str = ToStdString(aimrt_view);
  for (size_t i = 0; i < str.size(); ++i) {
    EXPECT_TRUE(str[i] == aimrt_view.str[i]);
  }
  EXPECT_EQ(str.size(), aimrt_view.len);
}

}  // namespace aimrt::util