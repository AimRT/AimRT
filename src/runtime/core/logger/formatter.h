// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cassert>
#include <cinttypes>
#include <functional>

#include "core/logger/log_data_wrapper.h"
#include "core/logger/log_level_tool.h"
#include "core/logger/os.h"
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
              format_handlers_.emplace_back(FormatTime);
              break;
            case 'Y':  // year (2024)
              estimated_size_ += 4;
              format_handlers_.emplace_back(FormatYear);
              break;
            case 'm':  // month (03)
              estimated_size_ += 2;
              format_handlers_.emplace_back(FormatMonth);
              break;
            case 'd':  // day (15)
              estimated_size_ += 2;
              format_handlers_.emplace_back(FormatDay);
              break;
            case 'H':  // hour (14)
              estimated_size_ += 2;
              format_handlers_.emplace_back(FormatHour);
              break;
            case 'M':  // minute (30)
              estimated_size_ += 2;
              format_handlers_.emplace_back(FormatMinute);
              break;
            case 'S':  // second (45)
              estimated_size_ += 2;
              format_handlers_.emplace_back(FormatSecond);
              break;
            case 'D':  // date only (2024-03-15)
              estimated_size_ += 10;
              format_handlers_.emplace_back(FormatDate);
              break;
            case 'T':  // clock only (14:30:45)
              estimated_size_ += 8;
              format_handlers_.emplace_back(FormatClock);
              break;
            case 'f':  // microseconds (123456)
              estimated_size_ += 6;
              format_handlers_.emplace_back(FormatMicroseconds);
              break;
            case 'A':  // weekay (Sunday)
              estimated_size_ += 9;
              format_handlers_.emplace_back(FormatWeekday);
              break;
            case 'a':  // weekay-short (Sun)
              estimated_size_ += 3;
              format_handlers_.emplace_back(FormatWeekdayShort);
              break;
            case 'l':  // log level (Info)
              estimated_size_ += 5;
              format_handlers_.emplace_back(FormatLevel);
              break;
            case 't':  // thread id (1234)
              estimated_size_ += 10;
              format_handlers_.emplace_back(FormatThreadId);
              break;
            case 'n':  // module name (test_module)
              estimated_size_ += 32;
              format_handlers_.emplace_back(FormatModule);
              break;
            case 'G':  // file name_short (test_module.cpp)
              estimated_size_ += 32;
              format_handlers_.emplace_back(FormatFileShort);
              break;
            case 'g':  // file name (/XX/YY/ZZ/test_module.cpp)
              estimated_size_ += 256;
              format_handlers_.emplace_back(FormatFile);
              break;
            case 'R':  // row number (20)
              estimated_size_ += 8;
              format_handlers_.emplace_back(FormatLine);
              break;
            case 'F':  // function name (TestFunc)
              estimated_size_ += 32;
              format_handlers_.emplace_back(FormatFunction);
              break;
            case 'v':  // message
              format_handlers_.emplace_back(FormatMessage);
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
  static void FormatTime(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetTimeStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format microseconds
  static void FormatMicroseconds(const LogDataWrapper& data, std::string& buffer) {
    constexpr int WIDTH = 6;

    char micro_str[WIDTH + 1];

    sprintf(micro_str, "%06" PRIu64, aimrt::common::util::GetTimestampUs(data.t) % 1000000);

    buffer.append(micro_str, WIDTH);
  }

  static void FormatYear(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetYearStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void FormatMonth(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetMonthStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void FormatDay(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetDayStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void FormatHour(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetHourStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void FormatMinute(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetMinuteStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void FormatSecond(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetSecondStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void FormatDate(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetDateStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  static void FormatClock(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetClockStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format weekday
  static void FormatWeekday(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetWeekDayStr(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format weekday-short
  static void FormatWeekdayShort(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(aimrt::common::util::GetWeekDayStrShort(std::chrono::system_clock::to_time_t(data.t)));
  }

  // format log level
  static void FormatLevel(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(LogLevelTool::GetLogLevelName(data.lvl));
  }

  // format message
  static void FormatMessage(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.log_data, data.log_data_size);
  }

  // format thread id
  static void FormatThreadId(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(std::to_string(data.thread_id));
  }

  // format module name
  static void FormatModule(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.module_name);
  }

  // format file name
  static void FormatFile(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.file_name);
  }

  // format file name (short)
  static void FormatFileShort(const LogDataWrapper& data, std::string& buffer) {
    // Linux
    if (sizeof(folder_seps) == 2) {
      const char* rv = std::strrchr(data.file_name, folder_seps[0]);
      const char* short_name = (rv != nullptr) ? rv + 1 : data.file_name;
      buffer.append(short_name);
    } else {
      // Windows
      if (data.file_name == nullptr) [[unlikely]] {
        return;
      }
      const std::reverse_iterator<const char*> begin(data.file_name + std::strlen(data.file_name));
      const std::reverse_iterator<const char*> end(data.file_name);

      const auto it = std::find_first_of(begin, end,
                                         std::begin(folder_seps),
                                         std::end(folder_seps) - 1);

      const char* short_name = (it != end) ? it.base() : data.file_name;
      buffer.append(short_name);
    }
  }

  // format line number
  static void FormatLine(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(std::to_string(data.line));
  }

  // format function name
  static void FormatFunction(const LogDataWrapper& data, std::string& buffer) {
    buffer.append(data.function_name);
  }

 private:
  using FormatHandler = std::function<void(const LogDataWrapper&, std::string&)>;
  std::vector<FormatHandler> format_handlers_;

  size_t estimated_size_ = 0;
};

}  // namespace aimrt::runtime::core::logger