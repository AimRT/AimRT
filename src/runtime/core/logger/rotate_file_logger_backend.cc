// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/logger/rotate_file_logger_backend.h"

#include <chrono>
#include <filesystem>
#include <map>
#include <mutex>
#include <regex>

#include "core/logger/log_level_tool.h"
#include "util/exception.h"
#include "util/format.h"
#include "util/string_util.h"
#include "util/time_util.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::logger::RotateFileLoggerBackend::Options> {
  using Options = aimrt::runtime::core::logger::RotateFileLoggerBackend::Options;

  static Node encode(const Options& rhs) {
    Node node;
    node["path"] = rhs.path;
    node["filename"] = rhs.filename;
    node["max_file_size_m"] = rhs.max_file_size_m;
    node["max_file_num"] = rhs.max_file_num;
    node["module_filter"] = rhs.module_filter;
    node["log_executor_name"] = rhs.log_executor_name;
    node["pattern"] = rhs.pattern;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["path"]) rhs.path = node["path"].as<std::string>();
    if (node["filename"]) rhs.filename = node["filename"].as<std::string>();
    if (node["max_file_size_m"])
      rhs.max_file_size_m = node["max_file_size_m"].as<uint32_t>();
    if (node["max_file_num"])
      rhs.max_file_num = node["max_file_num"].as<uint32_t>();
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

RotateFileLoggerBackend::~RotateFileLoggerBackend() {
  if (ofs_.is_open()) {
    ofs_.flush();
    ofs_.clear();
    ofs_.close();
  }
}

void RotateFileLoggerBackend::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  std::filesystem::path log_path(options_.path);
  base_file_name_ = (log_path / options_.filename).string();

  if (!(std::filesystem::exists(log_path) && std::filesystem::is_directory(log_path))) {
    std::filesystem::create_directories(log_path);
  }

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
    pattern_ = options_.pattern;
  }
  formatter_.SetPattern(pattern_);

  options_node = options_;

  run_flag_.store(true);
}

void RotateFileLoggerBackend::Log(const LogDataWrapper& log_data_wrapper) noexcept {
  try {
    if (!run_flag_.load()) [[unlikely]]
      return;

    if (!CheckLog(log_data_wrapper)) [[unlikely]]
      return;

    std::string log_data_str = formatter_.Format(log_data_wrapper);

    auto log_work = [this, log_data_str{std::move(log_data_str)}]() {
      if (!ofs_.is_open() || ofs_.tellp() > options_.max_file_size_m * 1024 * 1024) {
        if (!OpenNewFile()) return;
      }
      ofs_.write(log_data_str.data(), log_data_str.size()) << std::endl;
    };

    log_executor_.Execute(std::move(log_work));
  } catch (const std::exception& e) {
    fprintf(stderr, "Log get exception: %s\n", e.what());
  }
}

bool RotateFileLoggerBackend::OpenNewFile() {
  bool rename_flag = false;
  if (ofs_.is_open()) {
    rename_flag = (ofs_.tellp() > options_.max_file_size_m * 1024 * 1024);
    ofs_.flush();
    ofs_.clear();
    ofs_.close();
  }

  if (rename_flag && (std::filesystem::status(base_file_name_).type() == std::filesystem::file_type::regular)) {
    std::filesystem::rename(
        base_file_name_, base_file_name_ + "_" + std::to_string(GetNextIndex()));
  }

  ofs_.open(base_file_name_, std::ios::app);
  if (!ofs_.is_open()) {
    fprintf(stderr, "open log file %s failed.\n", base_file_name_.c_str());
    return false;
  }

  CleanLogFile();

  return true;
}

void RotateFileLoggerBackend::CleanLogFile() {
  if (options_.max_file_num == 0) return;

  std::filesystem::path log_dir = std::filesystem::path(base_file_name_).parent_path();

  std::map<uint32_t, std::string> log_files;

  const std::filesystem::directory_iterator end_itr;
  for (std::filesystem::directory_iterator itr(log_dir); itr != end_itr; ++itr) {
    const std::string& cur_log_file_name = itr->path().string();

    if (cur_log_file_name.size() <= base_file_name_.size() + 1) continue;
    if (cur_log_file_name.substr(0, base_file_name_.size() + 1) != (base_file_name_ + "_")) continue;

    const std::string& cur_log_file_name_suffix = cur_log_file_name.substr(base_file_name_.size() + 1);
    if (!aimrt::common::util::IsDigitStr(cur_log_file_name_suffix)) continue;

    uint32_t cur_idx = atoi(cur_log_file_name_suffix.c_str());
    log_files.emplace(cur_idx, cur_log_file_name);
  }

  if (log_files.size() <= options_.max_file_num) return;

  uint32_t del_num = log_files.size() - options_.max_file_num;
  for (auto& itr : log_files) {
    if (del_num == 0) break;
    std::filesystem::remove(itr.second);
    --del_num;
  }
}

uint32_t RotateFileLoggerBackend::GetNextIndex() {
  uint32_t idx = 1;
  std::filesystem::path log_dir =
      std::filesystem::path(base_file_name_).parent_path();

  const std::filesystem::directory_iterator end_itr;
  for (std::filesystem::directory_iterator itr(log_dir); itr != end_itr;
       ++itr) {
    const std::string& cur_log_file_name = itr->path().string();
    if (cur_log_file_name.size() <= base_file_name_.size() + 1) continue;
    if (cur_log_file_name.substr(0, base_file_name_.size() + 1) !=
        (base_file_name_ + "_"))
      continue;

    const std::string& cur_log_file_name_suffix =
        cur_log_file_name.substr(base_file_name_.size() + 1);
    if (!aimrt::common::util::IsDigitStr(cur_log_file_name_suffix)) continue;
    uint32_t cur_idx = atoi(cur_log_file_name_suffix.c_str());
    if (cur_idx >= idx) idx = cur_idx + 1;
  }

  return idx;
}

bool RotateFileLoggerBackend::CheckLog(const LogDataWrapper& log_data_wrapper) {
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
