// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <algorithm>
#include <concepts>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace aimrt::common::util {

template <typename T>
concept IterableType =
    requires(T t, typename T::value_type v) {
      { t.begin() } -> std::same_as<typename T::iterator>;
      { t.end() } -> std::same_as<typename T::iterator>;
      { t.size() } -> std::same_as<typename T::size_type>;
    };

template <typename T>
concept MapType =
    requires(T t, typename T::key_type k) {
      { t.begin() } -> std::same_as<typename T::iterator>;
      { t.end() } -> std::same_as<typename T::iterator>;
      { t.size() } -> std::same_as<typename T::size_type>;
      { t[k] } -> std::same_as<typename T::mapped_type&>;
    };

template <IterableType T>
std::string Container2Str(
    const T& t, const std::function<std::string(const typename T::value_type&)>& f) {
  std::stringstream ss;
  ss << "size = " << t.size() << '\n';
  if (!f) return ss.str();

  constexpr size_t kMaxLineLen = 32;

  size_t ct = 0;
  for (const auto& itr : t) {
    std::string obj_str = f(itr);
    if (obj_str.empty()) obj_str = "<empty string>";

    ss << "[index=" << ct << "]:";
    if (obj_str.length() > kMaxLineLen || obj_str.find('\n') != std::string::npos) {
      ss << '\n';
    }

    ss << obj_str << '\n';

    ++ct;
  }
  return ss.str();
}

template <IterableType T>
std::string Container2Str(const T& t) {
  return Container2Str(
      t,
      [](const typename T::value_type& obj) {
        std::stringstream ss;
        ss << obj;
        return ss.str();
      });
}

template <MapType T>
std::string Map2Str(const T& m,
                    const std::function<std::string(const typename T::key_type&)>& fkey,
                    const std::function<std::string(const typename T::mapped_type&)>& fval) {
  std::stringstream ss;
  ss << "size = " << m.size() << '\n';
  if (!fkey) return ss.str();

  constexpr size_t kMaxLineLen = 32;

  size_t ct = 0;
  for (const auto& itr : m) {
    std::string key_str = fkey(itr.first);
    if (key_str.empty()) key_str = "<empty string>";

    std::string val_str;
    if (fval) {
      val_str = fval(itr.second);
      if (val_str.empty()) val_str = "<empty string>";
    } else {
      val_str = "<unable to print>";
    }

    ss << "[index=" << ct << "]:\n  [key]:";
    if (key_str.length() > kMaxLineLen || key_str.find('\n') != std::string::npos) {
      ss << '\n';
    }

    ss << key_str << "\n  [val]:";
    if (val_str.length() > kMaxLineLen || val_str.find('\n') != std::string::npos) {
      ss << '\n';
    }

    ss << val_str << '\n';

    ++ct;
  }
  return ss.str();
}

template <MapType T>
std::string Map2Str(const T& m) {
  return Map2Str(
      m,
      [](const typename T::key_type& obj) {
        std::stringstream ss;
        ss << obj;
        return ss.str();
      },
      [](const typename T::mapped_type& obj) {
        std::stringstream ss;
        ss << obj;
        return ss.str();
      });
}

template <IterableType T>
bool CheckContainerEqual(const T& t1, const T& t2) {
  if (t1.size() != t2.size()) return false;

  auto len = t1.size();
  auto itr1 = t1.begin();
  auto itr2 = t2.begin();

  for (size_t ii = 0; ii < len; ++ii) {
    if (*itr1 != *itr2) return false;
    ++itr1;
    ++itr2;
  }
  return true;
}

template <IterableType T>
bool CheckContainerEqualNoOrder(const T& input_t1, const T& input_t2) {
  auto t1(input_t1);
  auto t2(input_t2);

  std::sort(t1.begin(), t1.end());
  std::sort(t2.begin(), t2.end());

  if (t1.size() != t2.size()) return false;

  auto len = t1.size();
  auto itr1 = t1.begin();
  auto itr2 = t2.begin();

  for (size_t ii = 0; ii < len; ++ii) {
    if (*itr1 != *itr2) return false;
    ++itr1;
    ++itr2;
  }
  return true;
}

template <MapType T>
bool CheckMapEqual(const T& map1, const T& map2) {
  if (map1.size() != map2.size()) return false;

  auto len = map1.size();
  auto itr1 = map1.begin();
  auto itr2 = map2.begin();

  for (size_t ii = 0; ii < len; ++ii) {
    if ((itr1->first != itr2->first) || (itr1->second != itr2->second))
      return false;
    ++itr1;
    ++itr2;
  }
  return true;
}

}  // namespace aimrt::common::util
