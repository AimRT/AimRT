// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace aimrt::common::util {

/**
 * @brief trim string
 *
 * @param[inout] s pending string
 * @return std::string& Reference to processed string s
 */
inline std::string& Trim(std::string& s) {
  if (s.empty()) return s;
  s.erase(s.find_last_not_of(' ') + 1);
  s.erase(0, s.find_first_not_of(' '));
  return s;
}

inline std::string_view& Trim(std::string_view& s) {
  if (s.empty()) return s;
  auto p = s.find_last_not_of(' ');
  s = s.substr(0, (p != std::string_view::npos) ? (p + 1) : 0);
  p = s.find_first_not_of(' ');
  s = s.substr((p != std::string_view::npos) ? p : s.size());
  return s;
}

/**
 * @brief parses a string like a=1&b=2&c=3 into map
 * @note
 * The principle of segmentation: If the substring between two vseps is included, it is divided into key and val based on msep.
 * @param[in] source string to be split
 * @param[in] vsep The separator between multiple kvs, cannot be null
 * @param[in] msep The delimiter inside a single kv, cannot be empty
 * @param[in] trim Whether to remove spaces in k and v
 * @param[in] clear Whether to remove the case where the key is empty
 * @return std::map<std::string, std::string> parsed map
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline std::map<StringType, StringType> SplitToMap(std::string_view source,
                                                   std::string_view vsep = "&",
                                                   std::string_view msep = "=",
                                                   bool trim = true,
                                                   bool clear = true) {
  std::map<StringType, StringType> result;

  if (source.empty() || vsep.empty() || msep.empty() || vsep == msep)
    return result;
  size_t v_pos_end = 0, v_pos_start = 0;
  do {
    v_pos_end = source.find(vsep, v_pos_start);
    if (v_pos_end == std::string_view::npos) v_pos_end = source.length();
    if (v_pos_end > v_pos_start) {
      auto kv_str = source.substr(v_pos_start, v_pos_end - v_pos_start);
      size_t msep_pos = kv_str.find(msep);
      if (msep_pos != std::string_view::npos) {
        auto first = kv_str.substr(0, msep_pos);
        auto second = kv_str.substr(msep_pos + msep.size());
        if (trim) {
          Trim(first);
          if (!(clear && first.empty())) {
            result[StringType(first)] = StringType(Trim(second));
          }
        } else {
          if (!(clear && first.empty())) {
            result[StringType(first)] = StringType(second);
          }
        }
      }
    }
    v_pos_start = v_pos_end + vsep.size();
  } while (v_pos_end < source.length());

  return result;
}

/**
 * @brief The string of the splicing map is a=1&b=2&c=3
 *
 * @param[in] kvmap std::map<std::string,std::string> structure
 * @param[in] vsep Separator between multiple kvs
 * @param[in] msep Delimiter inside a single kv
 * @return std::string String after splicing
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline std::string JoinMap(const std::map<StringType, StringType>& kvmap,
                           std::string_view vsep = "&",
                           std::string_view msep = "=") {
  std::stringstream result;
  for (auto itr = kvmap.begin(); itr != kvmap.end(); ++itr) {
    if (itr != kvmap.begin()) result << vsep;
    result << itr->first << msep << itr->second;
  }
  return result.str();
}

/**
 * @brief Get val data in map<std::string,std::string> with default value
 *
 * @param[in] m std::map<std::string,std::string> structure
 * @param[in] key The key to get data
 * @param[in] defval Default string, return defval when m does not contain the corresponding key.
 * @return std::string result
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline const StringType& GetMapItemWithDef(const std::map<StringType, StringType>& m,
                                           const StringType& key,
                                           const StringType& defval = "") {
  auto finditr = m.find(key);
  return (finditr != m.end()) ? (finditr->second) : defval;
}

/**
 * @brief Add kv field to s
 *
 * @param[inout] s pending string
 * @param[in] key key
 * @param[in] val val
 * @param[in] vsep Separator between multiple kvs
 * @param[in] msep Delimiter inside a single kv
 * @return std::string& Reference to string s after splicing
 */
inline std::string& AddKV(std::string& s,
                          std::string_view key,
                          std::string_view val,
                          std::string_view vsep = "&",
                          std::string_view msep = "=") {
  if (!s.empty()) s += vsep;
  s += key;
  s += msep;
  s += val;
  return s;
}

/**
 * @brief Get the val corresponding to the key from a string like a=1&b=2&c=3
 * @note
 * It is faster than splitting into map and then finding, but if there are multiple calls, it is recommended to split into map and then find
 * @param[in] str pending string
 * @param[in] key key, cannot be null, cannot contain vsep or msep, otherwise return empty result
 * @param[in] vsep The separator between multiple kvs, cannot be null
 * @param[in] msep The delimiter inside a single kv, cannot be empty
 * @param[in] trim Do not count spaces
 * @return std::string key corresponding to val
 */
inline std::string_view GetValueFromStrKV(std::string_view str,
                                          std::string_view key,
                                          std::string_view vsep = "&",
                                          std::string_view msep = "=",
                                          bool trim = true) {
  if (key.empty() || vsep.empty() || msep.empty() || vsep == msep) return "";
  if (key.find(vsep) != std::string_view::npos || key.find(msep) != std::string_view::npos)
    return "";

  size_t key_pos = str.find(key);
  while (key_pos != std::string_view::npos) {
    size_t msep_pos = str.find(msep, key_pos + key.size());
    if (msep_pos == std::string_view::npos) return "";

    size_t key_start_pos = str.rfind(vsep, key_pos);
    if (key_start_pos == std::string_view::npos)
      key_start_pos = 0;
    else
      key_start_pos += vsep.length();

    bool right_key_flag = false;
    if (trim) {
      auto real_key = str.substr(key_start_pos, msep_pos - key_start_pos);
      Trim(real_key);
      if (real_key == key) right_key_flag = true;
    } else {
      if ((key_start_pos == key_pos) && (msep_pos == (key_pos + key.size())))
        right_key_flag = true;
    }

    if (right_key_flag) {
      size_t val_start_pos = msep_pos + msep.size();
      size_t val_end_pos = str.find(vsep, val_start_pos);
      if (val_end_pos == std::string_view::npos) val_end_pos = str.length();

      if (trim) {
        auto re = str.substr(val_start_pos, val_end_pos - val_start_pos);
        return Trim(re);
      }
      return str.substr(val_start_pos, val_end_pos - val_start_pos);
    }

    key_pos = str.find(key, msep_pos + msep.length());
  }
  return "";
}

/**
 * @brief split string to vector
 *
 * @param[in] source pending string
 * @param[in] sep separator
 * @param[in] trim Whether to remove spaces for each result
 * @param[in] clear Whether to remove empty items
 * @return std::vector<std::string> segmentation result
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline std::vector<StringType> SplitToVec(std::string_view source,
                                          std::string_view sep,
                                          bool trim = true,
                                          bool clear = true) {
  std::vector<StringType> result;
  if (source.empty() || sep.empty()) return result;

  size_t pos_end = 0, pos_start = 0;
  do {
    pos_end = source.find(sep, pos_start);
    if (pos_end == std::string_view::npos) pos_end = source.length();

    auto sub_str = source.substr(pos_start, pos_end - pos_start);

    if (trim) Trim(sub_str);

    if (!(clear && sub_str.empty())) result.emplace_back(sub_str);

    pos_start = pos_end + sep.size();
  } while (pos_end < source.length());

  return result;
}

/**
 * @brief splicing vector to string
 *
 * @param[in] vec pending vector
 * @param[in] sep separator
 * @return std::string String after splicing
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline std::string JoinVec(const std::vector<StringType>& vec, std::string_view sep) {
  std::string result;
  for (auto itr = vec.begin(); itr != vec.end(); ++itr) {
    if (itr != vec.begin()) result += sep;
    result += *itr;
  }
  return result;
}

/**
 * @brief splits the string to set, automatically dereloads
 *
 * @param[in] source pending string
 * @param[in] sep separator
 * @param[in] trim Whether to remove spaces for each result
 * @param[in] clear Whether to remove empty items
 * @return std::set<std::string> segmentation result
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline std::set<StringType> SplitToSet(std::string_view source,
                                       std::string_view sep,
                                       bool trim = true,
                                       bool clear = true) {
  std::set<StringType> result;
  if (source.empty() || sep.empty()) return result;

  size_t pos_end = 0, pos_start = 0;
  do {
    pos_end = source.find(sep, pos_start);
    if (pos_end == std::string_view::npos) pos_end = source.length();

    auto sub_str = source.substr(pos_start, pos_end - pos_start);

    if (trim) Trim(sub_str);

    if (!(clear && sub_str.empty())) result.emplace(sub_str);

    pos_start = pos_end + sep.size();
  } while (pos_end < source.length());

  return result;
}

/**
 * @brief splicing set to string
 *
 * @param[in] st pending set
 * @param[in] sep separator
 * @return std::string String after splicing
 */
template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline std::string JoinSet(const std::set<StringType>& st, std::string_view sep) {
  std::string result;
  for (auto itr = st.begin(); itr != st.end(); ++itr) {
    if (itr != st.begin()) result += sep;
    result += *itr;
  }
  return result;
}

/**
 * @brief determines whether the key string is in the list string, such as "123" is in "123,456,789"
 * @note
 * It is faster than splitting into vector and then finding, but if there are multiple calls, it is recommended to split into vector and then find
 * @param[in] str list string
 * @param[in] key key string, cannot be empty, cannot contain sep
 * @param[in] dep separator of list, not be empty
 * @param[in] trim whether to ignore spaces
 * @return true key in list
 * @return false key is not in list
 */
inline bool CheckIfInList(std::string_view str,
                          std::string_view key,
                          std::string_view sep = ",",
                          bool trim = true) {
  if (key.empty() || sep.empty()) return false;
  if (key.find(sep) != std::string_view::npos) return false;

  size_t key_pos = str.find(key);
  while (key_pos != std::string_view::npos) {
    size_t key_start_pos = str.rfind(sep, key_pos);
    if (key_start_pos == std::string_view::npos)
      key_start_pos = 0;
    else
      key_start_pos += sep.length();

    size_t key_end_pos = str.find(sep, key_pos + key.length());
    if (key_end_pos == std::string_view::npos) key_end_pos = str.length();

    if (trim) {
      auto real_key = str.substr(key_start_pos, key_end_pos - key_start_pos);
      Trim(real_key);
      if (real_key == key) return true;
    } else {
      if ((key_start_pos == key_pos) && (key_end_pos == (key_pos + key.size())))
        return true;
    }

    key_pos = str.find(key, key_end_pos + sep.length());
  }

  return false;
}

/**
 * @brief comparison version, such as 6.1.1 6.2.8
 *
 * @param[in] ver1 version 1
 * @param[in] ver2 version 2
 * @return int Return 1 is greater than, return 0 is equal, return -1 is less than
 */
inline int CmpVersion(std::string_view ver1, std::string_view ver2) {
  const auto& version1_detail =
      SplitToVec<std::string_view>(ver1, ".", true, true);
  const auto& version2_detail =
      SplitToVec<std::string_view>(ver2, ".", true, true);

  size_t idx = 0;
  for (idx = 0; idx < version1_detail.size() && idx < version2_detail.size(); ++idx) {
    int ver1_num = 0, ver2_num = 0;
    auto [ptr1, ec1] = std::from_chars(version1_detail[idx].data(),
                                       version1_detail[idx].data() + version1_detail[idx].size(),
                                       ver1_num);
    auto [ptr2, ec2] = std::from_chars(version2_detail[idx].data(),
                                       version2_detail[idx].data() + version2_detail[idx].size(),
                                       ver2_num);
    if (ec1 != std::errc() || ec2 != std::errc() ||
        ptr1 != version1_detail[idx].data() + version1_detail[idx].size() ||
        ptr2 != version2_detail[idx].data() + version2_detail[idx].size()) {
      throw std::runtime_error("Invalid version format");
    }
    if (ver1_num < ver2_num)
      return -1;
    if (ver1_num > ver2_num)
      return 1;
  }
  if (idx == version1_detail.size() && idx == version2_detail.size()) {
    return 0;
  }
  return version1_detail.size() > version2_detail.size() ? 1 : -1;
}

/**
 * @brief Check whether the version is between the set versions
 * @note Need to ensure that start_ver<end_ver is. If end_ver is empty, set to 999.9.9.9
 * @param[in] ver version to be checked
 * @param[in] start_ver Start version
 * @param[in] end_ver end version
 * @return true between the incoming versions
 * @return false Not between the incoming versions
 */
inline bool CheckVersionInside(std::string_view ver,
                               std::string_view start_ver,
                               std::string_view end_ver) {
  return (CmpVersion(ver, start_ver.empty() ? "0.0.0.0" : start_ver) >= 0 &&
          CmpVersion(ver, end_ver.empty() ? "999.9.9.9" : end_ver) <= 0);
}

/**
 * @brief Change the specified string ov in a string to the string nv
 *
 * @param[inout] str string to be replaced
 * @param[in] ov substring to be replaced
 * @param[in] nv substring to replace
 * @return std::string& Reference to the replaced string str
 */
inline std::string& ReplaceString(std::string& str,
                                  std::string_view ov,
                                  std::string_view nv) {
  if (str.empty() || ov.empty()) return str;
  std::vector<size_t> vec_pos;
  size_t pos = 0, old_len = ov.size(), new_len = nv.size();
  while ((pos = str.find(ov, pos)) != std::string::npos) {
    vec_pos.emplace_back(pos);
    pos += old_len;
  }
  size_t& vec_len = pos = vec_pos.size();
  if (vec_len) {
    if (old_len == new_len) {
      for (size_t ii = 0; ii < vec_len; ++ii)
        memcpy(const_cast<char*>(str.c_str() + vec_pos[ii]), nv.data(),
               new_len);
    } else if (old_len > new_len) {
      char* p = const_cast<char*>(str.c_str()) + vec_pos[0];
      vec_pos.emplace_back(str.size());
      for (size_t ii = 0; ii < vec_len; ++ii) {
        memcpy(p, nv.data(), new_len);
        p += new_len;
        size_t cplen = vec_pos[ii + 1] - vec_pos[ii] - old_len;
        memmove(p, str.c_str() + vec_pos[ii] + old_len, cplen);
        p += cplen;
      }
      str.resize(p - str.c_str());
    } else {
      size_t diff = new_len - old_len;
      vec_pos.emplace_back(str.size());
      str.resize(str.size() + diff * vec_len);
      char* p = const_cast<char*>(str.c_str()) + str.size();
      for (size_t ii = vec_len - 1; ii < vec_len; --ii) {
        size_t cplen = vec_pos[ii + 1] - vec_pos[ii] - old_len;
        p -= cplen;
        memmove(p, str.c_str() + vec_pos[ii] + old_len, cplen);
        p -= new_len;
        memcpy(p, nv.data(), new_len);
      }
    }
  }
  return str;
}

/**
 * @brief determines whether a string is composed of numbers and letters
 *
 * @param[in] str string to be judged
 * @return true All are composed of numbers and letters
 * @return false Not all composed of numbers and letters
 */
inline bool IsAlnumStr(std::string_view str) {
  if (str.length() == 0) return false;
  for (const auto& c : str) {
    if (!isalnum(c)) return false;
  }
  return true;
}

/**
 * @brief determines whether the string is composed of numbers
 *
 * @param[in] str string to be judged
 * @return true All are composed of numbers
 * @return false Not all composed of numbers
 */
inline bool IsDigitStr(std::string_view str) {
  if (str.length() == 0) return false;
  for (const auto& c : str) {
    if (c > '9' || c < '0') return false;
  }
  return true;
}

/**
 * @brief Get the collection of keys in map
 *
 * @tparam KeyType The type of key in the map
 * @tparam ValType The type of val in the map
 * @param m Enter map
 * @return std::set<KeyType> collection of keys
 */
template <typename KeyType, typename ValType>
inline std::set<KeyType> GetMapKeys(const std::map<KeyType, ValType>& m) {
  std::set<KeyType> re;
  for (const auto& it : m) re.emplace(it.first);
  return re;
}

/**
 * @brief Draw a table
 *
 * @param table table data
 * @param with_header whether to draw the header
 * @return std::string result string, can be printed directly
 */

template <class StringType = std::string_view>
  requires(std::is_same_v<StringType, std::string_view> ||
           std::is_same_v<StringType, std::string>)
inline std::string DrawTable(
    const std::vector<std::vector<StringType>>& table, bool with_header = true) {
  size_t num_logical_rows = table.size();
  if (num_logical_rows == 0) return "<empty table>";

  auto split_lines = [](std::string_view sv) {
    std::vector<std::string_view> result;
    size_t start = 0;
    for (size_t i = 0; i < sv.size(); ++i) {
      if (sv[i] == '\n') {
        size_t end = i;
        if (end > start && sv[end - 1] == '\r') {
          end--;
        }
        result.push_back(sv.substr(start, end - start));
        start = i + 1;
      }
    }
    size_t end = sv.size();
    if (end > start && (sv[end - 1] == '\r' || sv[end - 1] == '\n')) {
      end--;
    }
    result.push_back(sv.substr(start, end - start));
    return result;
  };

  std::vector<size_t> column_width;

  for (const auto& row_item : table) {
    if (column_width.size() < row_item.size()) {
      column_width.resize(row_item.size(), 0);
    }
    for (size_t ii = 0; ii < row_item.size(); ++ii) {
      std::vector<std::string_view> lines = split_lines(row_item[ii]);
      size_t max_line_width_in_cell = 0;
      for (const auto& line : lines) {
        max_line_width_in_cell = std::max(max_line_width_in_cell, line.size());
      }

      column_width[ii] = std::max(column_width[ii], max_line_width_in_cell);
    }
  }

  if (column_width.empty() && num_logical_rows > 0) {
    return "<table with rows but no columns>";
  }

  for (auto& item : column_width) {
    item += 2;
  }

  size_t num_columns = column_width.size();
  std::stringstream result;

  auto draw_horizontal_line = [&]() {
    for (size_t jj = 0; jj < num_columns; ++jj) {
      result << "+" << std::setfill('-') << std::setw(column_width[jj]) << "-";
    }
    result << "+\n";
  };

  // first line
  draw_horizontal_line();

  for (size_t ii = 0; ii < num_logical_rows; ++ii) {
    size_t max_visual_lines_this_row = 1;
    std::vector<std::vector<std::string_view>> split_cells_this_row(num_columns);

    for (size_t jj = 0; jj < num_columns; ++jj) {
      if (table[ii].size() > jj) {
        split_cells_this_row[jj] = split_lines(table[ii][jj]);
        max_visual_lines_this_row = std::max(max_visual_lines_this_row, split_cells_this_row[jj].size());
      } else {
        split_cells_this_row[jj].push_back({});
      }
    }

    for (size_t line_idx = 0; line_idx < max_visual_lines_this_row; ++line_idx) {
      for (size_t jj = 0; jj < num_columns; ++jj) {
        std::string_view current_line_content = (line_idx < (size_t)split_cells_this_row[jj].size()) ? split_cells_this_row[jj][line_idx] : std::string_view{};

        std::string cell_output = " ";
        cell_output += current_line_content;

        result << "|" << std::left << std::setfill(' ')
               << std::setw(column_width[jj]) << cell_output;
      }
      result << "|\n";
    }
    // table header
    if (with_header && ii == 0) {
      draw_horizontal_line();
    }
  }

  // last line
  draw_horizontal_line();

  return result.str();
}

/**
 * @brief Replace the part of the string in the form of ${XXX_ENV} with the value of the corresponding environment variable
 *
 * @param input
 * @return std::string
 */
inline std::string ReplaceEnvVars(std::string_view input) {
  std::regex pattern(R"(\$\{([^}]+)\})");
  std::smatch match;
  std::string result(input);
  size_t cur_pos = 0;

  while (std::regex_search(result.cbegin() + cur_pos, result.cend(), match, pattern)) {
    std::string env_name = match[1].str();
    const char* env_val = std::getenv(env_name.c_str());
    if (env_val == nullptr) env_val = "";

    result.replace(cur_pos + match.position(0), match.length(0), env_val);

    cur_pos += (match.position(0) + strlen(env_val));
  }

  return result;
}

/**
 * @brief character to lowercase
 *
 * @param c characters
 * @return char lowercase characters
 */
inline char CharToLower(char c) { return static_cast<char>(tolower(c)); }

/**
 * @brief character to capitalize
 *
 * @param c characters
 * @return char capital characters
 */
inline char CharToUpper(char c) { return static_cast<char>(toupper(c)); }

/**
 * @brief string to lowercase
 *
 * @param str string
 * @return std::string& lowercase string
 */
inline std::string& StrToLower(std::string& str) {
  std::transform(str.begin(), str.end(), str.begin(), CharToLower);
  return str;
}

/**
 * @brief string to lowercase
 *
 * @param str string
 * @return std::string lowercase string
 */
inline std::string StrToLower(std::string_view str) {
  std::string result;
  result.resize(str.size());

  std::transform(str.begin(), str.end(), result.begin(), CharToLower);
  return result;
}

/**
 * @brief string to uppercase
 *
 * @param str string
 * @return std::string& capital string
 */
inline std::string& StrToUpper(std::string& str) {
  std::transform(str.begin(), str.end(), str.begin(), CharToUpper);
  return str;
}

/**
 * @brief string to uppercase
 *
 * @param str string
 * @return std::string capital string
 */
inline std::string StrToUpper(std::string_view str) {
  std::string result;
  result.resize(str.size());

  std::transform(str.begin(), str.end(), result.begin(), CharToUpper);
  return result;
}

/**
 * @brief Comparison of whether two strings are equal after ignoring upper and lower case
 *
 * @param str1
 * @param str2
 * @return true
 * @return false
 */
inline bool CheckIEqual(std::string_view str1, std::string_view str2) {
  if (str1.size() != str2.size()) return false;

  return std::equal(str1.begin(), str1.end(), str2.begin(),
                    [](char a, char b) { return std::tolower(a) == std::tolower(b); });
}

/**
 * @brief string is converted to title form, with capitalized first character of word
 *
 * @param str string
 * @return std::string& title form string
 */
inline std::string& StrToTitleCase(std::string& str) {
  std::string::iterator it = str.begin();
  *it = CharToUpper(*it);
  for (; it != str.end() - 1; ++it) {
    if (*it == ' ') {
      *(it + 1) = CharToUpper(*(it + 1));
    }
  }
  return str;
}

/**
 * @brief standardise a path string
 *
 * @param path
 * @return std::string&
 */
inline std::string& StandardisePath(std::string& path) {
  if (path.empty()) return (path = "/");
  std::replace(path.begin(), path.end(), '\\', '/');
  if (path[path.length() - 1] != '/') path += '/';
  return path;
}

/**
 * @brief judge if str start with pattern
 *
 * @param str
 * @param pattern
 * @param ignore_case if ignore the case
 * @return true
 * @return false
 */
inline bool StartsWith(std::string_view str,
                       std::string_view pattern,
                       bool ignore_case = false) {
  const size_t str_len = str.length();
  const size_t pattern_len = pattern.length();
  if (str_len < pattern_len || pattern_len == 0) return false;

  if (ignore_case) {
    for (size_t i = 0; i < pattern_len; ++i) {
      if (tolower(pattern[i]) != tolower(str[i])) return false;
    }
    return true;
  }
  for (size_t i = 0; i < pattern_len; ++i) {
    if (pattern[i] != str[i]) return false;
  }
  return true;
}

/**
 * @brief judge if str end with pattern
 *
 * @param str
 * @param pattern
 * @param ignore_case if ignore the case
 * @return true
 * @return false
 */
inline bool EndsWith(std::string_view str,
                     std::string_view pattern,
                     bool ignore_case = false) {
  const size_t str_len = str.length();
  const size_t pattern_len = pattern.length();
  if (str_len < pattern_len || pattern_len == 0) return false;

  const size_t& begin_pos = str_len - pattern_len;
  if (ignore_case) {
    for (size_t i = 0; i < pattern_len; ++i) {
      if (tolower(pattern[i]) != tolower(str[begin_pos + i])) return false;
    }
    return true;
  }
  for (size_t i = 0; i < pattern_len; i++) {
    if (pattern[i] != str[begin_pos + i]) return false;
  }
  return true;
}

/**
 * @brief fnv1a hash algorithm, 64 bits
 *
 * @param data
 * @param len
 * @return uint64_t hash value
 */
inline uint64_t Hash64Fnv1a(const char* data, size_t len) {
  constexpr uint64_t kPrime = 0x100000001b3;
  uint64_t hash = 0xcbf29ce484222325;
  for (size_t i = 0; i < len; ++i) {
    hash = (hash ^ static_cast<uint8_t>(data[i])) * kPrime;
  }
  return hash;
}

/**
 * @brief fnv1a hash algorithm, 32 bits
 *
 * @param data
 * @param len
 * @return uint32_t hash value
 */
inline uint32_t Hash32Fnv1a(const char* data, size_t len) {
  constexpr uint32_t kPrime = 0x1000193;
  uint32_t hash = 0x811c9dc5;
  for (size_t i = 0; i < len; ++i) {
    hash = (hash ^ static_cast<uint8_t>(data[i])) * kPrime;
  }
  return hash;
}

template <typename T>
inline std::string SSToString(const T& obj) {
  std::stringstream ss;
  ss << obj;
  return ss.str();
}

struct StringHash {
  using hash_type = std::hash<std::string_view>;
  using is_transparent = void;

  std::size_t operator()(const char* str) const { return hash_type{}(str); }
  std::size_t operator()(std::string_view str) const { return hash_type{}(str); }
  std::size_t operator()(std::string const& str) const { return hash_type{}(str); }
};

/**
 * @brief Safely calculates truncation position for UTF-8 string
 *
 * @param[in] utf8_str UTF-8 encoded string
 * @param[in] actual_length Actual byte length of the string
 * @param[in] desired_length Desired truncation length in bytes
 * @return size_t Safe truncation position in bytes [0, actual_length]
 */
inline size_t SafeUtf8TruncationLength(const char* utf8_str,
                                       size_t actual_length,
                                       size_t desired_length) noexcept {
  if (desired_length >= actual_length) return actual_length;

  size_t truncate_pos = desired_length;
  constexpr uint8_t CONT_BYTE_MASK = 0xC0;
  constexpr uint8_t CONT_BYTE_FLAG = 0x80;

  if ((utf8_str[truncate_pos] & CONT_BYTE_MASK) != CONT_BYTE_FLAG) {
    return truncate_pos;
  }

  while (truncate_pos > 0 &&
         ((utf8_str[truncate_pos] & CONT_BYTE_MASK) == CONT_BYTE_FLAG)) {
    --truncate_pos;
  }

  return truncate_pos;
}

}  // namespace aimrt::common::util
