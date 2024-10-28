// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>

#define GET_PATH(path, only_filename)            \
  ([](const char* p, bool flag) -> const char* { \
    if (!flag) return p;                         \
    const char* last = strrchr(p, '/');          \
    return last ? last + 1 : p;                  \
  })(path, only_filename)

namespace aimrt::runtime::core::logger {

// check pattern
[[nodiscard]] inline std::string Pattern(const std::string& pattern) {
  // define pattern map
  static const std::unordered_map<std::string_view, int> pattern_map{
      {"%T", 0},  // Datetime with microseconds
      {"%f", 1},  // microseconds
      {"%l", 2},  // level
      {"%t", 3},  // thread id
      {"%n", 4},  // module name
      {"%s", 5},  // file absolute path
      {"%S", 5},  // todo: file name
      {"%L", 6},  // row number
      {"%C", 7},  // column number
      {"%F", 8},  // function name
      {"%v", 9}   // content
  };

  // input pattern cant be empty
  if (pattern.empty()) [[unlikely]] {
    throw std::runtime_error("Pattern is empty");
  }

  std::string result;

  for (size_t i = 0; i < pattern.size(); ++i) {
    // if not %, add to result and continue
    if (pattern[i] != '%') {
      result += pattern[i];
      continue;
    }

    // pattern cant be end with %
    if (i + 1 >= pattern.size()) {
      throw std::runtime_error("Incomplete pattern at end of string");
    }

    // check each part of pattern is valid
    std::string_view pat = std::string_view(pattern).substr(i, 2);
    if (!pattern_map.contains(pat)) {
      throw std::runtime_error("Unknown logger pattern: " + std::string(pat));
    }

    // convert pattern to format string
    result += "{" + std::to_string(pattern_map.at(pat)) + "}";
    i++;
  }

  return result;
}

}  // namespace aimrt::runtime::core::logger