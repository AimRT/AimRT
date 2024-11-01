// Copyright (c) 2024 The AimRT Authors.
// AimRT is licensed under Mulan PSL v2.

#include "util/same_arg_trait.h"

#include <gtest/gtest.h>
#include <functional>
#include <string>

namespace aimrt::common::util {
namespace {
// NOLINTBEGIN(readability-named-parameter, performance-unnecessary-value-param)

void func1(int, double) {}
void func2(int, double) {}
void func3(double, int) {}
void func4() {}
void func5(std::string, int, double) {}
int func6(int, double) { return 0; }

int return_int(int, double) { return 0; }
double return_double(int, double) { return 0.0; }
std::string return_string(int, double) { return ""; }

TEST(SameArgTraitTest, NormalFunctions) {
  EXPECT_TRUE((SameArguments<decltype(func1), decltype(func2)>));
  EXPECT_FALSE((SameArguments<decltype(func1), decltype(func3)>));
  EXPECT_FALSE((SameArguments<decltype(func1), decltype(func4)>));
  EXPECT_FALSE((SameArguments<decltype(func1), decltype(func5)>));

  EXPECT_TRUE((SameArguments<decltype(func1), decltype(func6)>));
  EXPECT_TRUE((SameArguments<decltype(return_int), decltype(return_double)>));
  EXPECT_TRUE((SameArguments<decltype(return_int), decltype(return_string)>));
}

class TestClass {
 public:
  void method1(int, double) {}
  void method2(int, double) const {}
  int method3(int, double) { return 0; }  // NOLINT(readability-convert-member-functions-to-static)
  virtual void method4(int, double) {}

  void overloaded(int, double) {}
  void overloaded(double, int) {}
  void overloaded(int, double) const {}
};

TEST(SameArgTraitTest, MemberFunctions) {
  using Method1 = decltype(&TestClass::method1);
  using Method2 = decltype(&TestClass::method2);
  using Method3 = decltype(&TestClass::method3);
  using Method4 = decltype(&TestClass::method4);

  EXPECT_TRUE((SameArguments<Method1, Method2>));
  EXPECT_TRUE((SameArguments<Method1, Method3>));
  EXPECT_TRUE((SameArguments<Method1, Method4>));
  EXPECT_TRUE((SameArguments<Method1, decltype(func1)>));

  using Overload1 = void (TestClass::*)(int, double);
  EXPECT_TRUE((SameArguments<Overload1, Method1>));
}

struct FunctionObject1 {
  void operator()(int, double) {}
};

struct FunctionObject2 {
  void operator()(int, double) const {}
};

struct FunctionObject3 {
  void operator()(double, int) {}
};

TEST(SameArgTraitTest, FunctionObjects) {
  EXPECT_TRUE((SameArguments<FunctionObject1, FunctionObject2>));
  EXPECT_FALSE((SameArguments<FunctionObject1, FunctionObject3>));
  EXPECT_TRUE((SameArguments<FunctionObject1, decltype(func1)>));
}

auto lambda1 = [](int, double) {};
auto lambda2 = [](int, double) {};
auto lambda3 = [](double, int) {};
auto lambda4 = [](int, double) -> int { return 0; };
auto lambda_capture = [x = 0](int, double) {};

TEST(SameArgTraitTest, LambdaFunctions) {
  EXPECT_TRUE((SameArguments<decltype(lambda1), decltype(lambda2)>));
  EXPECT_FALSE((SameArguments<decltype(lambda1), decltype(lambda3)>));
  EXPECT_TRUE((SameArguments<decltype(lambda1), decltype(lambda4)>));
  EXPECT_TRUE((SameArguments<decltype(lambda1), decltype(lambda_capture)>));
  EXPECT_TRUE((SameArguments<decltype(lambda1), decltype(func1)>));
}

std::function<void(int, double)> std_func1;
std::function<int(int, double)> std_func2;
std::function<void(double, int)> std_func3;

TEST(SameArgTraitTest, StdFunction) {
  EXPECT_TRUE((SameArguments<decltype(std_func1), decltype(std_func2)>));
  EXPECT_FALSE((SameArguments<decltype(std_func1), decltype(std_func3)>));
  EXPECT_TRUE((SameArguments<decltype(std_func1), decltype(func1)>));
  EXPECT_TRUE((SameArguments<decltype(std_func1), decltype(lambda1)>));
}

void complex_func1(std::vector<int>, std::map<std::string, double>) {}
void complex_func2(std::vector<int>, std::map<std::string, double>) {}
void complex_func3(std::map<std::string, double>, std::vector<int>) {}

TEST(SameArgTraitTest, ComplexArgumentTypes) {
  EXPECT_TRUE((SameArguments<decltype(complex_func1), decltype(complex_func2)>));
  EXPECT_FALSE((SameArguments<decltype(complex_func1), decltype(complex_func3)>));
}

void ref_func1(int&, const double&) {}
void ref_func2(int&, const double&) {}
void ptr_func(int*, double*) {}

TEST(SameArgTraitTest, ReferenceAndPointerArguments) {
  EXPECT_TRUE((SameArguments<decltype(ref_func1), decltype(ref_func2)>));
  EXPECT_FALSE((SameArguments<decltype(ref_func1), decltype(ptr_func)>));
  EXPECT_FALSE((SameArguments<decltype(ref_func1), decltype(func1)>));
}

TEST(SameArgTraitTest, CrossTypeComparisons) {
  using Method1 = decltype(&TestClass::method1);
  using ConstMethod = decltype(&TestClass::method2);
  using VirtualMethod = decltype(&TestClass::method4);

  EXPECT_TRUE((SameArguments<decltype(func1), Method1>));
  EXPECT_TRUE((SameArguments<Method1, decltype(std_func1)>));
  EXPECT_TRUE((SameArguments<Method1, FunctionObject1>));
  EXPECT_TRUE((SameArguments<Method1, decltype(lambda1)>));

  EXPECT_TRUE((SameArguments<FunctionObject1, decltype(lambda1)>));
  EXPECT_TRUE((SameArguments<decltype(lambda1), FunctionObject1>));

  EXPECT_TRUE((SameArguments<ConstMethod, Method1>));
  EXPECT_TRUE((SameArguments<ConstMethod, decltype(std_func1)>));
  EXPECT_TRUE((SameArguments<ConstMethod, FunctionObject2>));

  EXPECT_TRUE((SameArguments<VirtualMethod, Method1>));
  EXPECT_TRUE((SameArguments<VirtualMethod, decltype(lambda1)>));

  EXPECT_TRUE((SameArguments<decltype(return_int), decltype(lambda4)>));
  EXPECT_TRUE((SameArguments<decltype(return_int), decltype(std_func2)>));

  EXPECT_TRUE((SameArguments<decltype(complex_func1), decltype(complex_func2)>));
  EXPECT_FALSE((SameArguments<decltype(complex_func1), decltype(complex_func3)>));

  EXPECT_FALSE((SameArguments<decltype(ref_func1), decltype(lambda1)>));
  EXPECT_FALSE((SameArguments<decltype(ref_func1), FunctionObject1>));
  EXPECT_FALSE((SameArguments<Method1, decltype(ref_func1)>));
  EXPECT_FALSE((SameArguments<decltype(ref_func1), decltype(ptr_func)>));
  EXPECT_FALSE((SameArguments<decltype(ptr_func), decltype(lambda1)>));
}

// NOLINTEND(readability-named-parameter, performance-unnecessary-value-param)
}  // namespace
}  // namespace aimrt::common::util