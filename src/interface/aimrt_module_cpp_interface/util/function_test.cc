// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include <array>

#include "aimrt_module_cpp_interface/util/function.h"

extern "C" typedef struct {
  void (*invoker)(void*);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} test_function_ops_1_t;

extern "C" typedef struct {
  uint32_t (*invoker)(void*, uint32_t);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} test_function_ops_2_t;

extern "C" typedef struct {
  int (*invoker)(void*, bool, double, uint64_t);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} test_function_ops_3_t;

extern "C" typedef struct {
  int (*invoker)(void*);
  void (*relocator)(void* from, void* to);
  void (*destroyer)(void* object);
} test_function_ops_4_t;

namespace aimrt::util {

uint32_t TestPlainFunc(bool, double, uint64_t) { return 42; }

class TestClass {
 public:
  static uint32_t foo(bool, double, uint64_t) { return 42; }

  uint32_t bar(bool, double, uint64_t) { return n; }
  uint32_t n = 0;
};

TEST(FUNCTION_TEST, base) {
  Function<test_function_ops_1_t> f;
  ASSERT_EQ(sizeof(f), 4 * sizeof(void*));
  ASSERT_FALSE(f);

  Function<test_function_ops_1_t> f2(nullptr);
  ASSERT_FALSE(f);
}

TEST(FUNCTION_TEST, Lambda) {
  Function<test_function_ops_2_t> f([](uint32_t in) { return in + 1; });
  ASSERT_EQ(f(42), 43);
}

TEST(FUNCTION_TEST, PlainFunc) {
  Function<test_function_ops_3_t> f(TestPlainFunc);
  ASSERT_EQ(f(false, 1.23, 888), 42);
}

TEST(FUNCTION_TEST, MemberMethod) {
  Function<test_function_ops_3_t> f(&TestClass::foo);
  ASSERT_EQ(f(false, 1.23, 888), 42);

  TestClass c{100};
  Function<test_function_ops_3_t> f2(
      std::bind(&TestClass::bar, &c, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  ASSERT_EQ(f2(false, 1.23, 888), 100);
}

TEST(FUNCTION_TEST, LargeFunctorTest) {
  Function<test_function_ops_4_t> f;
  char payload[1000];

  payload[999] = 42;
  f = [payload] { return payload[999]; };
  ASSERT_EQ(42, f());
}

TEST(FUNCTION_TEST, FunctorMoveTest) {
  struct OnlyCopyable {
    OnlyCopyable() : v(new std::vector<int>()) {}
    OnlyCopyable(const OnlyCopyable& oc) : v(new std::vector<int>(*oc.v)) {}
    ~OnlyCopyable() { delete v; }
    std::vector<int>* v;
  };

  OnlyCopyable payload;
  payload.v->resize(100, 12);

  Function<test_function_ops_4_t> f, f2;
  f = [payload] { return payload.v->back(); };
  f2 = std::move(f);
  ASSERT_EQ(12, f2());
}

TEST(FUNCTION_TEST, LargeFunctorMoveTest) {
  Function<test_function_ops_4_t> f, f2;
  std::array<std::vector<int>, 100> payload;

  payload.back().resize(10, 12);
  f = [payload] { return payload.back().back(); };
  f2 = std::move(f);
  ASSERT_EQ(12, f2());
}

TEST(FUNCTION_TEST, Capture) {
  Function<test_function_ops_1_t> f;
  int x = 0;

  f = [&x]() { x = 1; };
  f();

  ASSERT_EQ(1, x);
}

TEST(FUNCTION_TEST, Clear) {
  Function<test_function_ops_1_t> f = [] {};
  ASSERT_TRUE(f);

  f = nullptr;
  ASSERT_FALSE(f);

  std::function<void()> f1 = nullptr;
  Function<test_function_ops_1_t> f2 = f1;
  ASSERT_FALSE(f2);

  f = std::move(f1);
  ASSERT_FALSE(f);

  std::function<void()> f3 = []() {};
  f = std::move(f3);
  ASSERT_TRUE(f);

  Function<test_function_ops_1_t> f4([]() {});
  ASSERT_TRUE(f4);
}

TEST(FUNCTION_TEST, NativeHandle) {
  using TestFunctionType = Function<test_function_ops_2_t>;
  TestFunctionType f([](uint32_t in) { return in + 1; });
  ASSERT_TRUE(f);

  aimrt_function_base_t* f_base = f.NativeHandle();
  const auto* ops = static_cast<const test_function_ops_2_t*>(f_base->ops);
  ASSERT_NE(ops, nullptr);

  int ret = ops->invoker(&(f_base->object_buf), 41);
  ASSERT_EQ(ret, 42);

  TestFunctionType f2(f_base);
  ASSERT_FALSE(f);
  ASSERT_TRUE(f2);
  ret = f2(41);
  ASSERT_EQ(ret, 42);

  TestFunctionType f3 = [](uint32_t n) -> uint32_t { return 42; };
  ASSERT_TRUE(f3);
  ret = f3(41);
  ASSERT_EQ(ret, 42);

  TestFunctionType f4(std::move(f3));
  ASSERT_FALSE(f3);
  ASSERT_TRUE(f4);
  ret = f4(41);
  ASSERT_EQ(ret, 42);

  TestFunctionType f5 = [](uint32_t n) -> uint32_t { return 42; };
  ASSERT_TRUE(f5);
  ret = f5(41);
  ASSERT_EQ(ret, 42);

  TestFunctionType f6(std::move(f5));
  ASSERT_FALSE(f5);
  ASSERT_TRUE(f6);
  ret = f6(41);
  ASSERT_EQ(ret, 42);
}

}  // namespace aimrt::util
