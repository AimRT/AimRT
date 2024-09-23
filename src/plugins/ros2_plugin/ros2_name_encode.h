// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <string>
#include <string_view>

#include "util/url_encode.h"

namespace aimrt::plugins::ros2_plugin {

using aimrt::common::util::FromHex;
using aimrt::common::util::ToHex;

/**
 * @brief Ros2NameEncode
 *
 * @param[in] str string to be encoded
 * @param[in] up whether to encode to uppercase
 * @return std::string encoded string
 */
inline std::string Ros2NameEncode(std::string_view str, bool up = true) {
  std::string ret_str;
  ret_str.reserve(str.length() * 3);

  for (unsigned char c : str) {
    if (std::isalnum(c) || c == '/') {
      ret_str += static_cast<char>(c);
    } else {
      ret_str += '_';
      ret_str += static_cast<char>(ToHex(c >> 4, up));
      ret_str += static_cast<char>(ToHex(c & 0xF, up));
    }
  }
  return ret_str;
}

/**
 * @brief Ros2NameDecode
 *
 * @param[in] str string to be decoded
 * @return std::string decoded string
 */
inline std::string Ros2NameDecode(std::string_view str) {
  std::string ret_str;
  ret_str.reserve(str.length());

  for (const auto *it = str.begin(); it != str.end(); ++it) {
    if (*it == '_') {
      if (std::distance(it, str.end()) >= 3) {
        unsigned char high = FromHex(static_cast<unsigned char>(*(++it)));
        unsigned char low = FromHex(static_cast<unsigned char>(*(++it)));
        ret_str += static_cast<char>((high << 4) | low);
      } else {
        break;
      }
    } else {
      ret_str += *it;
    }
  }

  return ret_str;
}

}  // namespace aimrt::plugins::ros2_plugin