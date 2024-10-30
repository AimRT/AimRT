// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include "core/logger/log_data_wrapper.h"
#include "core/logger/log_level_tool.h"
#include "time_util.h"

namespace aimrt::runtime::core::logger {

class LogFormatter {
 public:
  std::string Format(const LogDataWrapper& log_data_wrapper) {
    std::string result;
    for (const auto& handler : format_handlers_) {
      handler(log_data_wrapper, result);
    }

    return result;
  }

  void SetPattern(std::string pattern) {
    format_handlers_.clear();

    size_t pos = 0;
    size_t last_text_pos = 0;

    while (pos < pattern.length()) {
      if (pattern[pos] == '%' && pos + 1 < pattern.length()) {
        // deal with normal text
        if (pos > 0 && pattern[pos - 1] != '%') {
          std::string text = pattern.substr(last_text_pos, pos - last_text_pos);
          if (!text.empty()) {
            format_handlers_.emplace_back(
                [text](const LogDataWrapper&, std::string& r) {
                  r.append(text);
                });
          }
        }

        // deal with format specifier
        switch (pattern[pos + 1]) {
          case 'T':  // time stamp without
            format_handlers_.emplace_back(format_time);
            break;
          case 'f':  // microseconds
            format_handlers_.emplace_back(format_microseconds);
            break;
          case 'l':  // log level
            format_handlers_.emplace_back(format_level);
            break;
          case 't':  // thread id
            format_handlers_.emplace_back(format_thread_id);
            break;
          case 'n':  // module name
            format_handlers_.emplace_back(format_module);
            break;
          case 's':  // file name_short
            format_handlers_.emplace_back(format_file_short);
          case 'g':  // file name
            format_handlers_.emplace_back(format_file);
            break;
          case 'L':  // row number
            format_handlers_.emplace_back(format_line);
            break;
          case 'C':  // column number
            format_handlers_.emplace_back(format_column);
            break;
          case 'F':  // function name
            format_handlers_.emplace_back(format_function);
            break;
          case 'v':  // message
            format_handlers_.emplace_back(format_message);
            break;
          default:
            format_handlers_.emplace_back(
                [c = pattern[pos + 1]](const LogDataWrapper&, std::string& r) {
                  r.push_back(c);
                });
            break;
        }
        pos += 2;
        last_text_pos = pos;
      } else {
        ++pos;
      }
    }

    // deal with the last text
    if (last_text_pos < pattern.length()) {
      std::string text = pattern.substr(last_text_pos);
      format_handlers_.emplace_back(
          [text](const LogDataWrapper&, std::string& r) {
            r.append(text);
          });
    }
  }

 private:
  // format time
  static void format_time(const LogDataWrapper& log_data_wrapper, std::string& result) {
    result.append(aimrt::common::util::GetTimeStr(
        std::chrono::system_clock::to_time_t(log_data_wrapper.t)));
  }

  // format log level
  static void format_level(const LogDataWrapper& log_data_wrapper, std::string& result) {
    result.append(LogLevelTool::GetLogLevelName(log_data_wrapper.lvl));
  }

  // format message
  static void format_message(const LogDataWrapper& log_data_wrapper, std::string& result) {
    result.append(log_data_wrapper.log_data, log_data_wrapper.log_data_size);
  }

  // format microseconds
  static void format_microseconds(const LogDataWrapper& data, std::string& result) {
    result.append(std::to_string(
        aimrt::common::util::GetTimestampUs(data.t) % 1000000));
  }

  // format thread id
  static void format_thread_id(const LogDataWrapper& data, std::string& result) {
    result.append(std::to_string(data.thread_id));
  }

  // format module name
  static void format_module(const LogDataWrapper& data, std::string& result) {
    result.append(data.module_name);
  }

  // format file name
  static void format_file(const LogDataWrapper& data, std::string& result) {
    result.append(data.file_name);
  }

  // format file name (short)
  static void format_file_short(const LogDataWrapper& data, std::string& result) {
    const char* last_slash = strrchr(data.file_name, '/');
    result.append("");
  }

  // format line number
  static void format_line(const LogDataWrapper& data, std::string& result) {
    result.append(std::to_string(data.line));
  }

  // format column number
  static void format_column(const LogDataWrapper& data, std::string& result) {
    result.append(std::to_string(data.column));
  }

  // format function name
  static void format_function(const LogDataWrapper& data, std::string& result) {
    result.append(data.function_name);
  }

  std::vector<std::function<void(const LogDataWrapper&, std::string&)>> format_handlers_;
};

}  // namespace aimrt::runtime::core::logger