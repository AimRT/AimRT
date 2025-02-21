// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cassert>
#include <functional>

#include "core/logger/log_data_wrapper.h"
#include "core/logger/log_level_tool.h"
#include "util/exception.h"
#include "util/time_util.h"

namespace aimrt::runtime::core::logger {

class LogFormatter {
 public:
  std::string Format(const LogDataWrapper& log_data_wrapper) {
    std::string buffer;
    buffer.reserve(log_data_wrapper.log_data_size + estimated_size_);  // reserve space for log data
    for (const auto& handler : format_handlers_) {
      handler(log_data_wrapper, buffer);
    }
    return buffer;
  }

  void SetPattern(const std::string& pattern) {
    AIMRT_ASSERT(!pattern.empty(), "Invalid argument: Logger's pattern cannot be empty");

    try {
      format_handlers_.clear();

      size_t pos = 0;
      size_t last_text_pos = 0;

      while (pos < pattern.length()) {
        if (pattern[pos] == '%' && pos + 1 < pattern.length()) {
          // deal with normal text
          if (pos > 0 && pattern[pos - 1] != '%') {
            auto text = std::string_view(pattern.data() + last_text_pos, pos - last_text_pos);
            if (!text.empty()) {
              estimated_size_ += text.length();
              format_handlers_.emplace_back(
                  [text = std::string(text)](auto&, std::string& buffer) {
                    buffer.append(text);
                  });
            }
          }

          // deal with format specifier
          switch (pattern[pos + 1]) {
            case 'c':  // data and time (2024-03-15 14:30:45)
              estimated_size_ += 19;
              format_handlers_.emplace_back(format_time);
              break;
            case 'Y':  // year (2024)
              estimated_size_ += 4;
              format_handlers_.emplace_back(format_year);
              break;
            case 'm':  // month (03)
              estimated_size_ += 2;
              format_handlers_.emplace_back(format_month);
              break;
            case 'd':  // day (15)
              estimated_size_ += 2;
              format_handlers_.emplace_back(format_day);
              break;
            case 'H':  // hour (14)
              estimated_size_ += 2;
              format_handlers_.emplace_back(format_hour);
              break;
            case 'M':  // minute (30)
              estimated_size_ += 2;
              format_handlers_.emplace_back(format_minute);
              break;
            case 'S':  // second (45)
              estimated_size_ += 2;
              format_handlers_.emplace_back(format_second);
              break;
            case 'D':  // date only (2024-03-15)
              estimated_size_ += 10;
              format_handlers_.emplace_back(format_date);
              break;
            case 'T':  // clock only (14:30:45)
              estimated_size_ += 8;
              format_handlers_.emplace_back(format_clock);
              break;
            case 'f':  // microseconds (123456)
              estimated_size_ += 6;
              format_handlers_.emplace_back(format_microseconds);
              break;
            case 'A':  // weekay (Sunday)
              estimated_size_ += 9;
              format_handlers_.emplace_back(format_weekday);
              break;
            case 'a':  // weekay-short (Sun)
              estimated_size_ += 3;
              format_handlers_.emplace_back(format_weekday_short);
              break;
            case 'l':  // log level (Info)
              estimated_size_ += 5;
              format_handlers_.emplace_back(format_level);
              break;
            case 't':  // thread id (1234)
              estimated_size_ += 10;
              format_handlers_.emplace_back(format_thread_id);
              break;
            case 'n':  // module name (test_module)
              estimated_size_ += 32;
              format_handlers_.emplace_back(format_module);
              break;
            case 'G':  // file name_short (test_module.cpp)
              estimated_size_ += 32;
              format_handlers_.emplace_back(format_file_short);
              break;
            case 'g':  // file name (/XX/YY/ZZ/test_module.cpp)
              estimated_size_ += 256;
              format_handlers_.emplace_back(format_file);
              break;
            case 'R':  // row number (20)
              estimated_size_ += 8;
              format_handlers_.emplace_back(format_line);
              break;
            case 'C':  // column number (20)
              estimated_size_ += 4;
              format_handlers_.emplace_back(format_column);
              break;
            case 'F':  // function name (TestFunc)
              estimated_size_ += 32;
              format_handlers_.emplace_back(format_function);
              break;
            case 'v':  // message
              format_handlers_.emplace_back(format_message);
              break;
            default:
              format_handlers_.emplace_back(
                  [c = pattern[pos + 1]](const LogDataWrapper&, std::string& buffer) {
                    buffer.push_back(c);
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
        std::string_view text(pattern.data() + last_text_pos, pattern.length() - last_text_pos);
        estimated_size_ += text.length();
        format_handlers_.emplace_back(
            [text = std::string(text)](const LogDataWrapper&, std::string& buffer) {
              buffer.append(text);
            });
      }
    } catch (const std::exception& e) {
      throw aimrt::common::util::AimRTException("Error in LogFormatter::SetPattern: " + std::string(e.what()));
    }
  }

 private:
  // format time
  static void format_time(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetTimeStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format microseconds
  static void format_microseconds(const LogDataWrapper& data, std::string& buffer) {
    constexpr int WIDTH = 6;

    thread_local char micro_str[WIDTH + 1] = {'0', '0', '0', '0', '0', '0', '\0'};

    sprintf(micro_str, "%06llu", aimrt::common::util::GetTimestampUs(data.t) % 1000000);

    buffer.append(micro_str, WIDTH);
  }

  static void format_year(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetYearStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void format_month(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetMonthStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void format_day(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetDayStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void format_hour(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetHourStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void format_minute(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetMinuteStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void format_second(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetSecondStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void format_date(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetDateStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void format_clock(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetClockStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format weekday
  static void format_weekday(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetWeekDayStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format weekday-short
  static void format_weekday_short(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetWeekDayStrShort(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format log level
  static void format_level(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(LogLevelTool::GetLogLevelName(data.lvl));
  }

  // format message
  static void format_message(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.log_data, data.log_data_size);
  }

  // format thread id
  static void format_thread_id(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(std::to_string(data.thread_id));
  }

  // format module name
  static void format_module(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.module_name);
  }

  // format file name
  static void format_file(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.file_name);
  }

  // format file name (short)
  static void format_file_short(const LogDataWrapper& data, std::string& buffer) {
    const char* last_slash = strrchr(data.file_name, '/');
    if (last_slash) {
      buffer.append(last_slash + 1);
    } else {
      buffer.append(data.file_name);
    }
  }

  // format line number
  static void format_line(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(std::to_string(data.line));
  }

  // format column number
  static void format_column(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(std::to_string(data.column));
  }

  // format function name
  static void format_function(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.function_name);
  }

 private:
  using FormatHandler = std::function<void(const LogDataWrapper&, std::string&)>;
  std::vector<FormatHandler> format_handlers_;

  size_t estimated_size_ = 0;
};

}  // namespace aimrt::runtime::core::logger