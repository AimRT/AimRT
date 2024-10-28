// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/logger/console_logger_backend.h"

#include <chrono>
#include <iostream>
#include <mutex>
#include <regex>

#include "core/logger/formatter.h"
#include "core/logger/log_level_tool.h"
#include "util/exception.h"
#include "util/format.h"
#include "util/time_util.h"

#if defined(_WIN32)
  #include <windows.h>

  #define CC_DEFAULT (FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED)

  #define CC_DBG (FOREGROUND_BLUE)
  #define CC_INF (FOREGROUND_GREEN)
  #define CC_WRN (FOREGROUND_GREEN | FOREGROUND_RED)
  #define CC_ERR (FOREGROUND_BLUE | FOREGROUND_RED)
  #define CC_FATAL \
    (BACKGROUND_RED | FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED)

HANDLE g_hConsole = INVALID_HANDLE_VALUE;

void set_console_window(void) {
  g_hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  if (g_hConsole == INVALID_HANDLE_VALUE) return;

  CONSOLE_FONT_INFOEX FontInfo;
  COORD dwSize;
  SetConsoleOutputCP(CP_UTF8);
  FontInfo.cbSize = sizeof(FontInfo);
  FontInfo.dwFontSize.X = 8;
  FontInfo.dwFontSize.Y = 16;
  wcsncpy(FontInfo.FaceName, L"Consolas", LF_FACESIZE);
  FontInfo.FontFamily = FF_DONTCARE;
  FontInfo.FontWeight = FW_BOLD;
  FontInfo.nFont = 1;
  SetCurrentConsoleFontEx(g_hConsole, FALSE, &FontInfo);
  dwSize.X = 80;
  dwSize.Y = 1024;
  SetConsoleScreenBufferSize(g_hConsole, dwSize);
  SetConsoleMode(g_hConsole, ENABLE_QUICK_EDIT_MODE | ENABLE_EXTENDED_FLAGS);
  // HMENU hMenu = GetSystemMenu(GetConsoleWindow(), FALSE);
  // EnableMenuItem(hMenu, SC_CLOSE, MF_GRAYED);

  return;
}

#else
  #define CC_NONE "\033[0m"
  #define CC_BLACK "\033[0;30m"
  #define CC_L_BLACK "\033[1;30m"
  #define CC_RED "\033[0;31m"
  #define CC_L_RED "\033[1;31m"
  #define CC_GREEN "\033[0;32m"
  #define CC_L_GREEN "\033[1;32m"
  #define CC_BROWN "\033[0;33m"
  #define CC_YELLOW "\033[1;33m"
  #define CC_BLUE "\033[0;34m"
  #define CC_L_BLUE "\033[1;34m"
  #define CC_PURPLE "\033[0;35m"
  #define CC_L_PURPLE "\033[1;35m"
  #define CC_CYAN "\033[0;36m"
  #define CC_L_CYAN "\033[1;36m"
  #define CC_GRAY "\033[0;37m"
  #define CC_WHITE "\033[1;37m"

  #define CC_BOLD "\033[1m"
  #define CC_UNDERLINE "\033[4m"
  #define CC_BLINK "\033[5m"
  #define CC_REVERSE "\033[7m"
  #define CC_HIDE "\033[8m"
  #define CC_CLEAR "\033[2J"

  #define CC_DBG CC_BLUE
  #define CC_INF CC_GREEN
  #define CC_WRN CC_YELLOW
  #define CC_ERR CC_PURPLE
  #define CC_FATAL CC_RED
#endif

namespace YAML {
template <>
struct convert<aimrt::runtime::core::logger::ConsoleLoggerBackend::Options> {
  using Options = aimrt::runtime::core::logger::ConsoleLoggerBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["color"] = rhs.print_color;
    node["module_filter"] = rhs.module_filter;
    node["log_executor_name"] = rhs.log_executor_name;
    node["pattern"] = rhs.pattern;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["color"]) rhs.print_color = node["color"].as<bool>();
    if (node["module_filter"])
      rhs.module_filter = node["module_filter"].as<std::string>();
    if (node["log_executor_name"])
      rhs.log_executor_name = node["log_executor_name"].as<std::string>();
    if (node["pattern"])
      rhs.pattern = node["pattern"].as<std::string>();

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::logger {

void ConsoleLoggerBackend::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

#if defined(_WIN32)
  if (options_.print_color) set_console_window();
#endif

  log_executor_ = get_executor_func_(options_.log_executor_name);
  if (!log_executor_) {
    throw aimrt::common::util::AimRTException(
        "Invalid log executor name: " + options_.log_executor_name);
  }

  if (!log_executor_.ThreadSafe()) {
    throw aimrt::common::util::AimRTException(
        "Log executor must be thread safe. Log executor name: " + options_.log_executor_name);
  }

  if (!options_.pattern.empty()) {
    pattern_ = Pattern(options_.pattern);
  }

  options_node = options_;

  run_flag_.store(true);
}

void ConsoleLoggerBackend::Log(const LogDataWrapper& log_data_wrapper) noexcept {
  try {
    if (!run_flag_.load()) [[unlikely]]
      return;

    if (!CheckLog(log_data_wrapper)) [[unlikely]]
      return;

    auto log_data_str = ::aimrt_fmt::format(
        aimrt_fmt::runtime(pattern_),
        aimrt::common::util::GetTimeStr(std::chrono::system_clock::to_time_t(log_data_wrapper.t)),
        (aimrt::common::util::GetTimestampUs(log_data_wrapper.t) % 1000000),
        LogLevelTool::GetLogLevelName(log_data_wrapper.lvl),
        log_data_wrapper.thread_id,
        log_data_wrapper.module_name,
        log_data_wrapper.file_name,
        log_data_wrapper.line,
        log_data_wrapper.column,
        log_data_wrapper.function_name,
        std::string_view(log_data_wrapper.log_data, log_data_wrapper.log_data_size));

    auto log_work = [this, lvl = log_data_wrapper.lvl, log_data_str{std::move(log_data_str)}]() {
      if (options_.print_color) {
#if defined(_WIN32)
        static constexpr WORD
            color_array[aimrt_log_level_t::AIMRT_LOG_LEVEL_OFF] = {
                0, CC_DBG, CC_INF, CC_WRN, CC_ERR, CC_FATAL};

        if (color_array[lvl] == 0) {
          std::cout << log_data_str;
        } else {
          SetConsoleTextAttribute(g_hConsole, color_array[lvl]);
          std::cout << log_data_str;
          SetConsoleTextAttribute(g_hConsole, CC_DEFAULT);
        }

#else
        static constexpr std::string_view
            kColorArray[aimrt_log_level_t::AIMRT_LOG_LEVEL_OFF] = {
                "", CC_DBG, CC_INF, CC_WRN, CC_ERR, CC_FATAL};

        if (kColorArray[lvl].empty()) {
          std::cout << log_data_str;
        } else {
          std::cout << kColorArray[lvl] << log_data_str << CC_NONE;
        }
#endif
      } else {
        std::cout << log_data_str;
      }
      std::cout << std::endl;
    };

    log_executor_.Execute(std::move(log_work));
  } catch (const std::exception& e) {
    fprintf(stderr, "Log get exception: %s\n", e.what());
  }
}

bool ConsoleLoggerBackend::CheckLog(const LogDataWrapper& log_data_wrapper) {
  {
    std::shared_lock lock(module_filter_map_mutex_);
    auto find_itr = module_filter_map_.find(log_data_wrapper.module_name);
    if (find_itr != module_filter_map_.end()) {
      return find_itr->second;
    }
  }

  bool if_log = false;

  try {
    if (std::regex_match(
            log_data_wrapper.module_name.begin(),
            log_data_wrapper.module_name.end(),
            std::regex(options_.module_filter, std::regex::ECMAScript))) {
      if_log = true;
    }
  } catch (const std::exception& e) {
    fprintf(stderr, "Regex get exception, expr: %s, string: %s, exception info: %s\n",
            options_.module_filter.c_str(), log_data_wrapper.module_name.data(), e.what());
  }

  std::unique_lock lock(module_filter_map_mutex_);
  module_filter_map_.emplace(log_data_wrapper.module_name, if_log);

  return if_log;
}
}  // namespace aimrt::runtime::core::logger